using System.Collections.Generic;
using UnityEngine;

public class ModelController : MonoBehaviour
{
    // 人物模型
    //物理
    protected Rigidbody rigidBody;

    private List<Vector3> posePointsList;

    private float tallHeadNeck, tallNeckSpine, tallSpineCrotch, tallThigh, tallShin;
    private Vector3 initPos;

    Animator animator;


    private Transform root, head, nose, left_eye, right_eye, left_ear, right_ear, left_shouler, right_shoulder,
                        left_elbow, right_elbow, left_wrist, right_wrist, left_thumb, right_thumb,
                        left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle,
                        left_heel, right_heel, left_index, right_index, left_upper_arm, right_upper_arm,
                        left_pinky, right_pinky, left_foot_index, right_foot_index, spine, neck;
    private Quaternion mid_root, mid_left_shoulder, mid_right_shoulder, mid_left_upper_arm, mid_left_elbow,
                        mid_left_hand, mid_right_hand, mid_right_upper_arm, mid_right_elbow,
        mid_left_hip, mid_left_knee, mid_left_foot, mid_right_hip, mid_right_knee, mid_right_foot,
        mid_spine, mid_neck;
    //public Transform nose;//鼻子，是为了计算脸部旋转的，我们是没有的，所以不用


    public void Start()
    {

        //Unity场景中的角色声明

        animator = this.GetComponent<Animator>();
        /////////////////////////////////////////////////// 骨骼定义 (从Unity库中获取骨骼定义)///////////////////////////////////////////////////

        //躯干
        root = animator.GetBoneTransform(HumanBodyBones.Hips);

        spine = animator.GetBoneTransform(HumanBodyBones.Spine);
        //chest = animator.GetBoneTransform(HumanBodyBones.Chest);
        //upperChest = animator.GetBoneTransform(HumanBodyBones.UpperChest);
        neck = animator.GetBoneTransform(HumanBodyBones.Neck);
        //头骨
        nose = animator.GetBoneTransform(HumanBodyBones.Head);
        //左臂
        left_upper_arm = animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);//左手肩膀
        left_elbow = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);//左手手肘
        left_wrist = animator.GetBoneTransform(HumanBodyBones.LeftHand);// 左手手腕
        left_thumb = animator.GetBoneTransform(HumanBodyBones.LeftThumbIntermediate);// 左手大拇指第1根指骨
        left_index = animator.GetBoneTransform(HumanBodyBones.LeftMiddleProximal); //左手中指第一节指骨
        left_pinky = animator.GetBoneTransform(HumanBodyBones.LeftLittleProximal);//左手小拇指的第1节指骨
        //右臂
        right_upper_arm = animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
        right_elbow = animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
        right_wrist = animator.GetBoneTransform(HumanBodyBones.RightHand);
        right_thumb = animator.GetBoneTransform(HumanBodyBones.RightThumbIntermediate);
        right_index = animator.GetBoneTransform(HumanBodyBones.RightMiddleProximal);
        right_pinky = animator.GetBoneTransform(HumanBodyBones.RightLittleProximal);
        //左腿
        left_hip = animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
        left_knee = animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg);//左膝
        left_ankle = animator.GetBoneTransform(HumanBodyBones.LeftFoot); //左脚脚踝
        left_foot_index = animator.GetBoneTransform(HumanBodyBones.LeftToes);// 左脚趾骨

        //右腿
        right_hip = animator.GetBoneTransform(HumanBodyBones.RightUpperLeg);//1
        right_knee = animator.GetBoneTransform(HumanBodyBones.RightLowerLeg);
        right_ankle = animator.GetBoneTransform(HumanBodyBones.RightFoot);
        right_foot_index = animator.GetBoneTransform(HumanBodyBones.RightToes);


        //initPos = root.position;
        /////////////////////////////////////////////////// 骨骼中间变换矩阵 ///////////////////////////////////////////////////
        // 当前旋转 = lookforward * 中间矩阵
        // 对于初始姿态，当前旋转就是初始旋转，结合骨骼方向和人体方向，求解各关节的中间矩阵
        Vector3 forward = TriangleNormal(left_upper_arm.position, right_hip.position, left_hip.position);
        // midLshoulder, midLelbow, midLhand, midRscapular, midRshoulder, midRelbow, midRhand, midLhip, midLknee, midLfoot, midLtoe, midRhip, midRknee, midRfoot, midRtoe        

        //以下是求每个节点的中间旋转矩阵

        // Root
        mid_root = Quaternion.Inverse(root.rotation) * Quaternion.LookRotation(forward);

        //躯干
        mid_spine = Quaternion.Inverse(spine.rotation) * Quaternion.LookRotation(spine.position - neck.position, forward);
        mid_neck = Quaternion.Inverse(neck.rotation) * Quaternion.LookRotation(neck.position - nose.position, forward);

        // 左臂：上臂，胳膊肘，手腕
        mid_left_upper_arm = Quaternion.Inverse(left_upper_arm.rotation) * Quaternion.LookRotation(left_upper_arm.position - left_elbow.position, forward);
        mid_left_elbow = Quaternion.Inverse(left_elbow.rotation) * Quaternion.LookRotation(left_elbow.position - left_wrist.position, forward);

        mid_left_hand = Quaternion.Inverse(left_wrist.rotation) * Quaternion.LookRotation(
            left_thumb.position - left_index.position,
            TriangleNormal(left_wrist.position, left_pinky.position, left_index.position));


        // 右臂
        mid_right_upper_arm = Quaternion.Inverse(right_upper_arm.rotation) * Quaternion.LookRotation(right_upper_arm.position - right_elbow.position, forward);
        mid_right_elbow = Quaternion.Inverse(right_elbow.rotation) * Quaternion.LookRotation(right_elbow.position - right_wrist.position, forward);
        mid_right_hand = Quaternion.Inverse(right_wrist.rotation) * Quaternion.LookRotation(
            right_thumb.position - right_index.position,
            TriangleNormal(right_thumb.position, right_index.position, right_pinky.position));
        // 左腿
        mid_left_hip = Quaternion.Inverse(left_hip.rotation) * Quaternion.LookRotation(left_hip.position - left_knee.position, forward);
        mid_left_knee = Quaternion.Inverse(left_knee.rotation) * Quaternion.LookRotation(left_knee.position - left_ankle.position, forward);
        mid_left_foot = Quaternion.Inverse(left_ankle.rotation) * Quaternion.LookRotation(left_ankle.position - left_foot_index.position, left_knee.position - left_ankle.position);
        // 右腿
        mid_right_hip = Quaternion.Inverse(right_hip.rotation) * Quaternion.LookRotation(right_hip.position - right_knee.position, forward);
        mid_right_knee = Quaternion.Inverse(right_knee.rotation) * Quaternion.LookRotation(right_knee.position - right_ankle.position, forward);
        mid_right_foot = Quaternion.Inverse(right_ankle.rotation) * Quaternion.LookRotation(right_ankle.position - right_foot_index.position, right_knee.position - right_ankle.position);

    }

    // 计算三角形法向量
    Vector3 TriangleNormal(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 d1 = a - b;
        Vector3 d2 = a - c;

        Vector3 dd = Vector3.Cross(d1, d2);
        dd.Normalize();//单位向量

        return dd;
    }

    public void UpdateModelPose(List<Vector3> pred3D)
    {
        //this.gameObject.transform.Rotate(Vector3.up, 180.0f);

        //////////////////////  更新位置 //////////////////////
        //计算根节点位置
        float tallShin = (Vector3.Distance(pred3D[25], pred3D[27]) + Vector3.Distance(pred3D[26], pred3D[28])) / 2.0f;//左、右小腿
        float tallThigh = (Vector3.Distance(pred3D[23], pred3D[25]) + Vector3.Distance(pred3D[24], pred3D[26])) / 2.0f;//左、右大腿
        float tallUnity = (Vector3.Distance(left_hip.position, left_knee.position) + Vector3.Distance(left_knee.position, left_ankle.position)) / 2.0f +
            (Vector3.Distance(right_hip.position, right_knee.position) + Vector3.Distance(right_knee.position, right_ankle.position)) / 2.0f;

        Vector3 rootpos = ((pred3D[23] + pred3D[24]) / 2) * (tallUnity / (tallThigh + tallShin));
        float x = rootpos.x;
        rootpos = rootpos - new Vector3(x, 0, 0);
        root.localPosition = rootpos;
        //root.localPosition = ((pred3D[23] + pred3D[24]) / 2) * (tallUnity / (tallThigh + tallShin));//24是根

        //输出根节点坐标
        //Debug.Log("根节点坐标位置" + (pred3D[23] + pred3D[24]) / 2);
        //Debug.Log("比例：" + (tallUnity / (tallThigh + tallShin)));
        //Debug.Log("最终位置" + root.position);

        //////////////////////  更新旋转 //////////////////////

        /*当前旋转Rotation=当前关节的lookrotation×Quaternion.Inverse(对齐矩阵)*/

        Vector3 root_cal = (pred3D[11] + pred3D[12] + 2 * pred3D[23] + 2 * pred3D[24]) / 6;
        Vector3 neck_cal = (pred3D[10] + pred3D[9] + 2 * pred3D[12] + 2 * pred3D[11]) / 6;
        Vector3 spine_cal = (pred3D[11] + pred3D[12] + pred3D[23] + pred3D[24]) / 4;

        //Vector3 forward = TriangleNormal(root_cal, pred3D[24], pred3D[23]);//朝向

        Vector3 forward = TriangleNormal(pred3D[12], pred3D[24], pred3D[23]);

        // Root
        root.rotation = Quaternion.LookRotation(forward) * Quaternion.Inverse(mid_root);
        // 躯干
        spine.rotation = Quaternion.LookRotation(spine_cal - neck_cal, forward) * Quaternion.Inverse(mid_spine);
        //spine.rotation = Quaternion.LookRotation(spine_cal - neck_cal, forward) * Quaternion.Inverse(mid_spine);
        //neck.rotation = Quaternion.LookRotation(neck_cal - pred3D[0], forward) * Quaternion.Inverse(mid_neck);
        // 左臂
        left_upper_arm.rotation = Quaternion.LookRotation(pred3D[11] - pred3D[13], forward) * Quaternion.Inverse(mid_left_upper_arm);
        left_elbow.rotation = Quaternion.LookRotation(pred3D[13] - pred3D[15], forward) * Quaternion.Inverse(mid_left_elbow);
        left_wrist.rotation = Quaternion.LookRotation(
                    pred3D[15] - pred3D[17],
                    TriangleNormal(pred3D[17], pred3D[19], pred3D[15])) * Quaternion.Inverse(mid_left_hand);
        // 右臂
        right_upper_arm.rotation = Quaternion.LookRotation(pred3D[12] - pred3D[14], forward) * Quaternion.Inverse(mid_right_upper_arm);
        right_elbow.rotation = Quaternion.LookRotation(pred3D[14] - pred3D[16], forward) * Quaternion.Inverse(mid_right_elbow);
        right_wrist.rotation = Quaternion.LookRotation(
            pred3D[16] - pred3D[18],
            TriangleNormal(pred3D[16], pred3D[20], pred3D[18])) * Quaternion.Inverse(mid_right_hand);

        //// 左腿
        left_hip.rotation = Quaternion.LookRotation(pred3D[23] - pred3D[25], forward) * Quaternion.Inverse(mid_left_hip);
        left_knee.rotation = Quaternion.LookRotation(pred3D[25] - pred3D[27], forward) * Quaternion.Inverse(mid_left_knee);
        //// 右腿
        right_hip.rotation = Quaternion.LookRotation(pred3D[24] - pred3D[26], forward) * Quaternion.Inverse(mid_right_hip);
        right_knee.rotation = Quaternion.LookRotation(pred3D[26] - pred3D[28], forward) * Quaternion.Inverse(mid_right_knee);

    }

    private void Update()
    {
        if (posePointsList != null) UpdateModelPose(posePointsList);
    }

    // 更新姿势
    public void UpdateModelPosePointsList(List<Vector3> pred3D)
    {
        string points = "";
        foreach (var i in pred3D)
        {
            points += (i.ToString() + " ");
        }
        //Debug.Log("Pose List: " + points);
        //Debug.Log("Pose List");
        //Debug.Log(pred3D);

        posePointsList = pred3D;
    }
}
