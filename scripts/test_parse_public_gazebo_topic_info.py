from pathlib import Path
import pytest
from parse_public_gazebo_topic_info import parse
def test_unique_block_rejects_cross_block_and_bad_gid(tmp_path):
 p=tmp_path/'info'; p.write_text('Type: sensor_msgs/msg/Image\nPublisher count: 1\nPublisher GID:\n  aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\nNode name: owner\nNode namespace: /\n')
 parse(p,'sensor_msgs/msg/Image','owner')
 p.write_text(p.read_text()+'Publisher GID:\n  bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n')
 with pytest.raises(ValueError):parse(p,'sensor_msgs/msg/Image','owner')
