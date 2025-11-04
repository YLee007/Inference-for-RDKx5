# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

int32 index
builtin_interfaces/Time time_stamp
uint32 height
uint32 width
uint32 data_size
uint32 step
uint8[12] encoding  # 这个字段用于标识图像格式
uint8[6220800] data # 存储图像数据，大小为6220800字节，适用于1080P图像的NV12格式数据

uint32 MAX_SIZE=6220800
