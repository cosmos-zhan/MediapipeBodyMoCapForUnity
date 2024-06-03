using Mediapipe.Unity.Sample;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mediapipe.Unity.PoseTracking
{

    public class MyPoseTrackingSolution : ImageSourceSolution<MyPoseTrackingGraph>
    {

        //[SerializeField] private RectTransform _worldAnnotationArea;
        //[SerializeField] private DetectionAnnotationController _poseDetectionAnnotationController;
        //[SerializeField] private PoseLandmarkListAnnotationController _poseLandmarksAnnotationController;
        //[SerializeField] private PoseWorldLandmarkListAnnotationController _poseWorldLandmarksAnnotationController;
        [SerializeField] private ModelController _modelController;
        //[SerializeField] private BodyAngle poseHandler;
        //[SerializeField] private MaskAnnotationController _segmentationMaskAnnotationController;
        //[SerializeField] private NormalizedRectAnnotationController _roiFromLandmarksAnnotationController;

        List<Vector3> currentFrame;

        int counter = 0;
        bool status = true;

        public List<Vector3> CurrentFrame
        {
            get => currentFrame;
            set => currentFrame = value;
        }

        // 参数选项
        public MyPoseTrackingGraph.ModelComplexity modelComplexity
        {
            get => graphRunner.modelComplexity;
            set => graphRunner.modelComplexity = value;
        }

        public bool smoothLandmarks
        {
            get => graphRunner.smoothLandmarks;
            set => graphRunner.smoothLandmarks = value;
        }

        public bool enableSegmentation
        {
            get => graphRunner.enableSegmentation;
            set => graphRunner.enableSegmentation = value;
        }

        public bool smoothSegmentation
        {
            get => graphRunner.smoothSegmentation;
            set => graphRunner.smoothSegmentation = value;
        }

        public float minDetectionConfidence
        {
            get => graphRunner.minDetectionConfidence;
            set => graphRunner.minDetectionConfidence = value;
        }

        public float minTrackingConfidence
        {
            get => graphRunner.minTrackingConfidence;
            set => graphRunner.minTrackingConfidence = value;
        }


        // 启动，绑定graph的输出监听
        protected override void OnStartRun()
        {
            if (!runningMode.IsSynchronous())
            {
                graphRunner.OnPoseDetectionOutput += OnPoseDetectionOutput;
                graphRunner.OnPoseLandmarksOutput += OnPoseLandmarksOutput;
                graphRunner.OnPoseWorldLandmarksOutput += OnPoseWorldLandmarksOutput;
                graphRunner.OnSegmentationMaskOutput += OnSegmentationMaskOutput;
                graphRunner.OnRoiFromLandmarksOutput += OnRoiFromLandmarksOutput;
            }

            var imageSource = ImageSourceProvider.ImageSource;
            //SetupAnnotationController(_poseDetectionAnnotationController, imageSource);
            //SetupAnnotationController(_poseLandmarksAnnotationController, imageSource);
            //SetupAnnotationController(_poseWorldLandmarksAnnotationController, imageSource);
            //SetupAnnotationController(_segmentationMaskAnnotationController, imageSource);
            //_segmentationMaskAnnotationController.InitScreen(imageSource.textureWidth, imageSource.textureHeight);
            //SetupAnnotationController(_roiFromLandmarksAnnotationController, imageSource);
        }

        // 将图像输入传给graph
        protected override void AddTextureFrameToInputStream(TextureFrame textureFrame)
        {
            graphRunner.AddTextureFrameToInputStream(textureFrame);
        }

        // 获取graph的同步输出
        protected override IEnumerator WaitForNextValue()
        {
            var task = graphRunner.WaitNextAsync();
            yield return new WaitUntil(() => task.IsCompleted);

            var result = task.Result;

            // 此处获取graph的同步输出
            //_poseDetectionAnnotationController.DrawNow(poseDetection);
            //_poseLandmarksAnnotationController.DrawNow(poseLandmarks);
            //_poseWorldLandmarksAnnotationController.DrawNow(result.poseWorldLandmarks);
            //_segmentationMaskAnnotationController.DrawNow(segmentationMask);
            //_roiFromLandmarksAnnotationController.DrawNow(roiFromLandmarks);
        }

        private void OnPoseDetectionOutput(object stream, OutputStream<Detection>.OutputEventArgs eventArgs)
        {
            //_poseDetectionAnnotationController.DrawLater(eventArgs.value);
        }

        private void OnPoseLandmarksOutput(object stream, OutputStream<NormalizedLandmarkList>.OutputEventArgs eventArgs)
        {
            var packet = eventArgs.packet;
            var value = packet == null ? default : packet.Get(NormalizedLandmarkList.Parser);
            if (value != null)
            {
                List<Vector3> curpos = new List<Vector3>();
                var normalizedLandmarkList = value;
                var landmarks = normalizedLandmarkList.Landmark;
                //Debug.Log("Pose World Landmarks");
                //Debug.Log(normalizedLandmarkList);
                foreach (var landmark in landmarks)
                {
                    var pos = new Vector3(landmark.X, 1 - landmark.Y, landmark.Z);
                    curpos.Add(pos);
                }

                currentFrame = curpos;
                //poseHandler.MotionRecognition(curpos, ref counter, ref status);
            }
        }

        private void OnPoseWorldLandmarksOutput(object stream, OutputStream<LandmarkList>.OutputEventArgs eventArgs)
        {
            var packet = eventArgs.packet;
            var value = packet == null ? default : packet.Get(LandmarkList.Parser);
            if (value != null)
            {
                List<Vector3> curpos = new List<Vector3>();
                var normalizedLandmarkList = value;
                var landmarks = normalizedLandmarkList.Landmark;
                Debug.Log("Pose World Landmarks");
                Debug.Log(normalizedLandmarkList);
                foreach (var landmark in landmarks)
                {
                    var pos = new Vector3(landmark.X, 1 - landmark.Y, landmark.Z);
                    curpos.Add(pos);
                }
                _modelController.UpdateModelPosePointsList(curpos);
            }
            //_poseWorldLandmarksAnnotationController.DrawLater(value);

        }

        private void OnSegmentationMaskOutput(object stream, OutputStream<ImageFrame>.OutputEventArgs eventArgs)
        {
            //_segmentationMaskAnnotationController.DrawLater(eventArgs.value);
        }

        private void OnRoiFromLandmarksOutput(object stream, OutputStream<NormalizedRect>.OutputEventArgs eventArgs)
        {
            //_roiFromLandmarksAnnotationController.DrawLater(eventArgs.value);
        }

    }
}
