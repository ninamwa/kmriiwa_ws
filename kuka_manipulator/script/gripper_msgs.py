# Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
# Copyright 2019 Norwegian University of Science and Technology.
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

from enum import Enum
class GripperMsg(Enum):
    Activation = b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30"
    ActivationRequest = b"\x09\x03\x07\xD0\x00\x01\x85\xCF"
    ActivationComplete = "09030200005985"

    StatusResponse= b"\x09\x03\x07\xD0\x00\x03\x04\x0E"
    CloseRequest = b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\x7F\x7F\x22\x49"
    OpenRequest = b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19"

    NOTMOVING = -1
    MOVING = 0
    OBJECT_OPENING = 1
    OBJECT_CLOSING = 2
    REQUESTEDPOSITION = 3