import pytest
import probe_public_gazebo_gt_transport as p
def row(n=1): return {'frame_id':'front_rgbd_depth_optical_frame','stamp_ns':n,'width':848,'height':480,'encoding':'rgb8','publisher_gid':'a'*64,'topic_types':{**p.TOPICS,'rgb_type':'sensor_msgs/msg/Image','semantic_type':'sensor_msgs/msg/Image','instance_type':'sensor_msgs/msg/Image','camera_info_type':'sensor_msgs/msg/CameraInfo'}}
def test_ten_frame_probe_is_exact_and_rejects_bad_gt_readiness():
 assert p.validate([row(i) for i in range(1,11)])['status']=='GT_TRANSPORT_PROBE_VERIFIED'
 with pytest.raises(ValueError): p.validate([row(i) for i in range(1,10)])
 bad=[row(i) for i in range(1,11)];bad[4]['encoding']='mono8'
 with pytest.raises(ValueError):p.validate(bad)
