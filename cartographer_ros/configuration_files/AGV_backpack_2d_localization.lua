-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "AGV_backpack_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2, --最大保存子圖數,存定位模式透過子圖進行定位,但只需要當前和上一個子圖就好
}
POSE_GRAPH.optimize_every_n_nodes = 15 --數字太小cpu會爆炸 (20幀一個子圖,子圖建構完成要閉環檢測),設置為0就是禁用global slam
return options
