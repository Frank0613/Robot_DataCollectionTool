import os
import glob
import random
from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
import robot_config

class ObjectSpawner:
    def __init__(self, world):
        self.world = world
        self.spawned_objects = [] # 追蹤已生成的物件名稱
        
        # 搜尋資料夾下所有的 .usd 檔案
        # 確保 robot_config.OBJECTS_DIR 已經在 config 中定義
        search_path = os.path.join(robot_config.OBJECTS_DIR, "*.usd")
        self.available_usds = glob.glob(search_path)
        print(f"[ObjectSpawner] Found {len(self.available_usds)} USD files in {robot_config.OBJECTS_DIR}")

    def respawn(self):
        # 1. 清除舊物件 (清除本次執行期間生成的)
        for name in self.spawned_objects:
            prim_path = f"/World/{name}"
            if self.world.scene.object_exists(name):
                self.world.scene.remove_object(name) # 從 Scene Registry 移除
            if is_prim_path_valid(prim_path):
                delete_prim(prim_path) # 從 Stage 移除
        self.spawned_objects = []

        if not self.available_usds:
            print("[Warning] No USD files found to spawn!")
            return

        # 2. 隨機選取 5 個物件
        count = min(5, len(self.available_usds))
        if len(self.available_usds) >= 5:
            selected_files = random.sample(self.available_usds, k=count)
        else:
            # 如果檔案不夠，允許重複選取
            selected_files = random.choices(self.available_usds, k=5)

        # 3. 在 5 個點生成
        for i, usd_path in enumerate(selected_files):
            # 取得 target point 的位置
            point_path = f"{robot_config.SPAWN_POINT_ROOT}/point_0{i+1}"
            
            # 獲取點的世界座標
            if is_prim_path_valid(point_path):
                position, orientation = XFormPrim(point_path).get_world_pose()
            else:
                print(f"[Warning] Spawn point not found: {point_path}")
                continue

            # 定義新物件的路徑與名稱
            obj_name = f"spawned_obj_{i}"
            obj_prim_path = f"/World/{obj_name}"
            
            # [防禦性刪除] 即使沒有紀錄，也要檢查路徑上是否有殘留物件
            if is_prim_path_valid(obj_prim_path):
                delete_prim(obj_prim_path)

            # 加入參考 (Reference)
            add_reference_to_stage(usd_path=usd_path, prim_path=obj_prim_path)

            # 包裝成 XFormPrim (使用 XForm 避免巢狀剛體錯誤)
            spawned_prim = XFormPrim(
                prim_path=obj_prim_path,
                name=obj_name,
                position=position,
                orientation=orientation
            )
            
            self.world.scene.add(spawned_prim)
            self.spawned_objects.append(obj_name)
        
        print(f"[ObjectSpawner] Spawned {len(self.spawned_objects)} objects.")