# Python: python/commands_config.py
def get_command_dict(instance):
    return {
        "地図作成": instance._make_map,
        "地図で作成": instance._make_map,
        "て作成": instance._make_map,
        "ダウンロード": instance._map_download,
        "アップロード": instance._map_upload_and_sort,
        "作成終了": instance._stop_making_map_downloading,
        "作成を終了": instance._stop_making_map_downloading,
        # "保存": instance._save_map,
        "着いてきて": instance._detect_and_follow,
        "ついてきて": instance._detect_and_follow,
        "終了": instance._stop_follow,
        "止まって": instance._stop_follow,
        "停止": instance._stop_follow,
        "休んで": instance._sit_down,
        "座って": instance._sit_down,
        "立って": instance._stand_up,
        "立ち上がって": instance._stand_up,
        "立ち上がれ": instance._stand_up,
        "お辞儀": instance._spot_bow_pose,
        "おじぎ": instance._spot_bow_pose,
        "を辞儀": instance._spot_bow_pose,
        "廃棄": instance._spot_thinning_pose,
        "を廃棄": instance._spot_thinning_pose,
        # "しゃがんで": instance._spot_squat_pose,
        "しゃがんで": instance._spot_squat,
        "初期化": instance._initialize,
        "記憶": instance._create_waypoint,
        "曲": instance._create_waypoint,
        "スタート": instance._navigate_to_first_position,
        "戻れ": instance._navigate_to_last_position,
        "戻って": instance._navigate_to_last_position,
        "ナビゲート": instance._navigate_route_all,
        "保存場所": instance._navigate_to_unique_position,
        "トラック": instance._navigate_to_unique_position,
        "捨ててきて": instance._spot_thinning,
        "を捨ててきて": instance._spot_thinning,
        "出てきて": instance._spot_thinning,
        "ドッキング": instance._spot_docking,
        "解除": instance._spot_undocking,
        "充電": instance._spot_find_dock,
        "回って": instance._spot_rotate,
        "まわって": instance._spot_rotate,
        "回転": instance._spot_rotate,
        # "トラック": instance._spot_thinning,
        "進め": instance._go_to_front,
        "進んで": instance._go_to_front,
        "すすんで": instance._go_to_front,
        "下がって": instance._go_to_back,
        "さがって": instance._go_to_back,
        "後ろに": instance._go_to_back,
        "うしろに": instance._go_to_back,
        "前": instance._go_front_little,
        "後ろ": instance._go_back_little,
        "右": instance._go_right_little,
        "左": instance._go_left_little,
        "うしろ": instance._go_back_little,
        "リセット": instance._reset_audio,
        "プログラム終了": instance._exit_program,
        "プログラム修了": instance._exit_program,
    }

def get_web_commands_dict(instance):
    return {
        'sit': instance._sit_down,           # 座る
        'stand': instance._stand_up,         # 立つ
        'follow': instance._detect_and_follow, # 追跡
        'stop': instance._stop_follow,       # 停止
        'memory': instance._spot_memory_truck, # 記憶
        'docking': instance._spot_docking,   # ドッキング
        'undocking': instance._spot_undocking, # ドッキング解除
        'bow': instance._spot_pose_bow,      # お辞儀
        'squat': instance._spot_squat_pose,  # しゃがむ
        'lie_down': instance._spot_squat,  # 伏せる
        'truck': instance._navigate_to_unique_position,
        'thinning': instance._spot_thinning,  # 廃棄
        'back': instance._navigate_to_last_position, # 元の場所に戻る
        'charge': instance._spot_find_dock,  # 充電
        'finish_charge': instance._spot_finish_dock, # 充電終了
        'forward': instance._go_to_front,  # 前進
        'backward': instance._go_to_back,  # 後退
        'rotate': instance._spot_rotate,  # 回転
    }
    
def get_keyboard_commands_dict(instance):
    return {
        # "t": instance._make_map,
        # "d": instance._map_download,
        # "u": instance._map_upload_and_sort,
        "r": instance._sit_down,
        "t": instance._stand_up,
        "q": instance._exit_program,
        "w": instance._go_front_little,
        "a": instance._go_left_little,
        "d": instance._go_right_little,
        "s": instance._go_back_little,
        "reset": instance._reset_audio,  # 新たに「reset」コマンドを追加
    }
