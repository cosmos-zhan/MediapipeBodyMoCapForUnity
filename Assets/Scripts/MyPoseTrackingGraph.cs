using Google.Protobuf;
using Mediapipe.Unity.Sample;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;

namespace Mediapipe.Unity.PoseTracking
{
    public readonly struct PoseTrackingResult
    {
        public readonly Detection poseDetection;
        public readonly NormalizedLandmarkList poseLandmarks;
        public readonly LandmarkList poseWorldLandmarks;
        public readonly ImageFrame segmentationMask;
        public readonly NormalizedRect roiFromLandmarks;

        public PoseTrackingResult(Detection poseDetection, NormalizedLandmarkList poseLandmarks, LandmarkList poseWorldLandmarks, ImageFrame segmentationMask, NormalizedRect roiFromLandmarks)
        {
            this.poseDetection = poseDetection;
            this.poseLandmarks = poseLandmarks;
            this.poseWorldLandmarks = poseWorldLandmarks;
            this.segmentationMask = segmentationMask;
            this.roiFromLandmarks = roiFromLandmarks;
        }
    }

    public class MyPoseTrackingGraph : GraphRunner
    {
        // 参数
        public enum ModelComplexity
        {
            Lite = 0,
            Full = 1,
            Heavy = 2,
        }

        public ModelComplexity modelComplexity = ModelComplexity.Full;
        public bool smoothLandmarks = true;
        public bool enableSegmentation = true;
        public bool smoothSegmentation = true;

        private float _minDetectionConfidence = 0.5f;
        public float minDetectionConfidence
        {
            get => _minDetectionConfidence;
            set => _minDetectionConfidence = Mathf.Clamp01(value);
        }

        private float _minTrackingConfidence = 0.5f;
        public float minTrackingConfidence
        {
            get => _minTrackingConfidence;
            set => _minTrackingConfidence = Mathf.Clamp01(value);
        }

        // 声明输入流的名称
        private const string _InputStreamName = "input_video";


        // 声明输出流和输出流的名称
        private OutputStream<Detection> _poseDetectionStream;
        private OutputStream<NormalizedLandmarkList> _poseLandmarksStream;
        private OutputStream<LandmarkList> _poseWorldLandmarksStream;
        private OutputStream<ImageFrame> _segmentationMaskStream;
        private OutputStream<NormalizedRect> _roiFromLandmarksStream;

        private const string _PoseDetectionStreamName = "pose_detection";
        private const string _PoseLandmarksStreamName = "pose_landmarks";
        private const string _PoseWorldLandmarksStreamName = "pose_world_landmarks";
        private const string _SegmentationMaskStreamName = "segmentation_mask";
        private const string _RoiFromLandmarksStreamName = "roi_from_landmarks";


        // 初始化配置
        protected override void ConfigureCalculatorGraph(CalculatorGraphConfig config)
        {

            _poseDetectionStream = new OutputStream<Detection>(calculatorGraph, _PoseDetectionStreamName, true);
            _poseLandmarksStream = new OutputStream<NormalizedLandmarkList>(calculatorGraph, _PoseLandmarksStreamName, true);
            _poseWorldLandmarksStream = new OutputStream<LandmarkList>(calculatorGraph, _PoseWorldLandmarksStreamName, true);
            _segmentationMaskStream = new OutputStream<ImageFrame>(calculatorGraph, _SegmentationMaskStreamName, true);
            _roiFromLandmarksStream = new OutputStream<NormalizedRect>(calculatorGraph, _RoiFromLandmarksStreamName, true);


            using (var validatedGraphConfig = new ValidatedGraphConfig())
            {
                validatedGraphConfig.Initialize(config);

                var extensionRegistry = new ExtensionRegistry() { TensorsToDetectionsCalculatorOptions.Extensions.Ext, ThresholdingCalculatorOptions.Extensions.Ext };
                var cannonicalizedConfig = validatedGraphConfig.Config(extensionRegistry);
                var tensorsToDetectionsCalculators = cannonicalizedConfig.Node.Where((node) => node.Calculator == "TensorsToDetectionsCalculator").ToList();
                var thresholdingCalculators = cannonicalizedConfig.Node.Where((node) => node.Calculator == "ThresholdingCalculator").ToList();

                foreach (var calculator in tensorsToDetectionsCalculators)
                {
                    if (calculator.Options.HasExtension(TensorsToDetectionsCalculatorOptions.Extensions.Ext))
                    {
                        var options = calculator.Options.GetExtension(TensorsToDetectionsCalculatorOptions.Extensions.Ext);
                        options.MinScoreThresh = minDetectionConfidence;
                        Logger.LogInfo(TAG, $"Min Detection Confidence = {minDetectionConfidence}");
                    }
                }

                foreach (var calculator in thresholdingCalculators)
                {
                    if (calculator.Options.HasExtension(ThresholdingCalculatorOptions.Extensions.Ext))
                    {
                        var options = calculator.Options.GetExtension(ThresholdingCalculatorOptions.Extensions.Ext);
                        options.Threshold = minTrackingConfidence;
                        Logger.LogInfo(TAG, $"Min Tracking Confidence = {minTrackingConfidence}");
                    }
                }
                calculatorGraph.Initialize(cannonicalizedConfig);
            }
        }

        // 定义异步监听的接口
        public event EventHandler<OutputStream<Detection>.OutputEventArgs> OnPoseDetectionOutput
        {
            add => _poseDetectionStream.AddListener(value, timeoutMicrosec);
            remove => _poseDetectionStream.RemoveListener(value);
        }

        public event EventHandler<OutputStream<NormalizedLandmarkList>.OutputEventArgs> OnPoseLandmarksOutput
        {
            add => _poseLandmarksStream.AddListener(value, timeoutMicrosec);
            remove => _poseLandmarksStream.RemoveListener(value);
        }

        public event EventHandler<OutputStream<LandmarkList>.OutputEventArgs> OnPoseWorldLandmarksOutput
        {
            add => _poseWorldLandmarksStream.AddListener(value, timeoutMicrosec);
            remove => _poseWorldLandmarksStream.RemoveListener(value);
        }

        public event EventHandler<OutputStream<ImageFrame>.OutputEventArgs> OnSegmentationMaskOutput
        {
            add => _segmentationMaskStream.AddListener(value, timeoutMicrosec);
            remove => _segmentationMaskStream.RemoveListener(value);
        }

        public event EventHandler<OutputStream<NormalizedRect>.OutputEventArgs> OnRoiFromLandmarksOutput
        {
            add => _roiFromLandmarksStream.AddListener(value, timeoutMicrosec);
            remove => _roiFromLandmarksStream.RemoveListener(value);
        }

        // 定义获取同步的接口
        public async Task<PoseTrackingResult> WaitNextAsync()
        {
            var results = await WhenAll(
              _poseDetectionStream.WaitNextAsync(),
              _poseLandmarksStream.WaitNextAsync(),
              _poseWorldLandmarksStream.WaitNextAsync(),
              _segmentationMaskStream.WaitNextAsync(),
              _roiFromLandmarksStream.WaitNextAsync()
            );
            AssertResult(results);

            _ = TryGetValue(results.Item1.packet, out var poseDetection, (packet) =>
            {
                return packet.Get(Detection.Parser);
            });
            _ = TryGetValue(results.Item2.packet, out var poseLandmarks, (packet) =>
            {
                return packet.Get(NormalizedLandmarkList.Parser);
            });
            _ = TryGetValue(results.Item3.packet, out var poseWorldLandmarks, (packet) =>
            {
                return packet.Get(LandmarkList.Parser);
            });
            _ = TryGetValue(results.Item4.packet, out var segmentationMask);
            _ = TryGetValue(results.Item5.packet, out var roiFromLandmarks, (packet) =>
            {
                return packet.Get(NormalizedRect.Parser);
            });

            return new PoseTrackingResult(poseDetection, poseLandmarks, poseWorldLandmarks, segmentationMask, roiFromLandmarks);
        }

        // 启动
        public override void StartRun(ImageSource imageSource)
        {
            // 此处进行输出流的启动 和 CalculatorGraph的StartRun
            _poseDetectionStream.StartPolling();
            _poseLandmarksStream.StartPolling();
            _poseWorldLandmarksStream.StartPolling();
            _segmentationMaskStream.StartPolling();
            _roiFromLandmarksStream.StartPolling();
            // calculatorGraph.StartRun().AssertOk();
            StartRun(BuildSidePacket(imageSource));
        }

        // 释放
        public override void Stop()
        {
            //关闭流
            base.Stop();
            _poseDetectionStream?.Dispose();
            _poseDetectionStream = null;
            _poseLandmarksStream?.Dispose();
            _poseLandmarksStream = null;
            _poseWorldLandmarksStream?.Dispose();
            _poseWorldLandmarksStream = null;
            _segmentationMaskStream?.Dispose();
            _segmentationMaskStream = null;
            _roiFromLandmarksStream?.Dispose();
            _roiFromLandmarksStream = null;
        }

        // 定义输入接口，用于将输入源传给输入流
        public void AddTextureFrameToInputStream(TextureFrame textureFrame)
        {
            AddTextureFrameToInputStream(_InputStreamName, textureFrame);
        }


        // 加载坐标检测的数据文件
        protected override IList<WaitForResult> RequestDependentAssets()
        {
            return new List<WaitForResult> {
                WaitForAsset("pose_detection.bytes"),
                WaitForPoseLandmarkModel(),
          };
        }

        // 用于切换模型复杂度
        private WaitForResult WaitForPoseLandmarkModel()
        {
            switch (modelComplexity)
            {
                case ModelComplexity.Lite: return WaitForAsset("pose_landmark_lite.bytes");
                case ModelComplexity.Full: return WaitForAsset("pose_landmark_full.bytes");
                case ModelComplexity.Heavy: return WaitForAsset("pose_landmark_heavy.bytes");
                default: throw new InternalException($"Invalid model complexity: {modelComplexity}");
            }
        }

        // 构造SidePacket
        private PacketMap BuildSidePacket(ImageSource imageSource)
        {
            var sidePacket = new PacketMap();

            SetImageTransformationOptions(sidePacket, imageSource);

            // TODO: refactoring
            // The orientation of the output image must match that of the input image.
            var isInverted = CoordinateSystem.ImageCoordinate.IsInverted(imageSource.rotation);
            var outputRotation = imageSource.rotation;
            var outputHorizontallyFlipped = !isInverted && imageSource.isHorizontallyFlipped;
            var outputVerticallyFlipped = (!runningMode.IsSynchronous() && imageSource.isVerticallyFlipped) ^ (isInverted && imageSource.isHorizontallyFlipped);

            if ((outputHorizontallyFlipped && outputVerticallyFlipped) || outputRotation == RotationAngle.Rotation180)
            {
                outputRotation = outputRotation.Add(RotationAngle.Rotation180);
                outputHorizontallyFlipped = !outputHorizontallyFlipped;
                outputVerticallyFlipped = !outputVerticallyFlipped;
            }

            sidePacket.Emplace("output_rotation", Packet.CreateInt((int)outputRotation));
            sidePacket.Emplace("output_horizontally_flipped", Packet.CreateBool(outputHorizontallyFlipped));
            sidePacket.Emplace("output_vertically_flipped", Packet.CreateBool(outputVerticallyFlipped));

            Debug.Log($"output_rotation = {outputRotation}, output_horizontally_flipped = {outputHorizontallyFlipped}, output_vertically_flipped = {outputVerticallyFlipped}");

            sidePacket.Emplace("model_complexity", Packet.CreateInt((int)modelComplexity));
            sidePacket.Emplace("smooth_landmarks", Packet.CreateBool(smoothLandmarks));
            sidePacket.Emplace("enable_segmentation", Packet.CreateBool(enableSegmentation));
            sidePacket.Emplace("smooth_segmentation", Packet.CreateBool(smoothSegmentation));

            Debug.Log($"Model Complexity = {modelComplexity}");
            Debug.Log($"Smooth Landmarks = {smoothLandmarks}");
            Debug.Log($"Enable Segmentation = {enableSegmentation}");
            Debug.Log($"Smooth Segmentation = {smoothSegmentation}");

            return sidePacket;
        }
    }

}
