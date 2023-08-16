import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from PIL import Image as PILImage
from io import BytesIO
import maskrcnn_pb2
import maskrcnn_pb2_grpc
import grpc

bridge = CvBridge()

if __name__ == '__main__':
    rospy.init_node('test_node')
    image = rospy.wait_for_message('/usb_cam/image_raw', Image)
    image_cv = bridge.imgmsg_to_cv2(image, "rgb8")
    image_np = np.array(image_cv)

    # 将image_np转换为jpeg格式的二进制数据
    image_pil = PILImage.fromarray(image_np)
    buf = BytesIO()
    image_pil.save(buf, format='jpeg')
    image_pil.save('test.jpg')
    image_binary = buf.getvalue()
    
    with grpc.insecure_channel('192.168.31.2:50051', options=[
        ('grpc.max_receive_message_length', 100*1024*1024),
    ]) as channel:
        stub = maskrcnn_pb2_grpc.MaskRCNNStub(channel)
        response = stub.maskrcnn(maskrcnn_pb2.MaskRCNNRequest(b_image=image_binary))

        # # class_names = np.frombuffer(response.b_class_names, dtype=str)
        # b_rois = np.frombuffer(response.b_rois, dtype=np.int32)
        # b_class_ids = np.frombuffer(response.b_class_ids, dtype=np.int32)
        # b_scores = np.frombuffer(response.b_scores, dtype=np.float32)
        # b_masks = np.frombuffer(response.b_masks, dtype=bool)

        def bytes2nparray(bytes):
            np_bytes = BytesIO(bytes)
            return np.load(np_bytes, allow_pickle=True)

        class_names = bytes2nparray(response.b_class_names)
        rois = bytes2nparray(response.b_rois)
        class_ids = bytes2nparray(response.b_class_ids)
        scores = bytes2nparray(response.b_scores)
        masks = bytes2nparray(response.b_masks)

        a = 3
    
    
