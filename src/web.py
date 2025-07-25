import os
import glob
import logging
import datetime
from typing import Optional
from flask import Flask, jsonify, render_template, send_from_directory

class RobotState:
    """ロボットの状態を管理するクラス"""
    def __init__(self):
        self.tracking = False
        self.map_recording = False
        self.going_save = False
        self.returning = False
        self.initializing = False
        self.current_state = "待機状態"
    
    def update(self, *, tracking=False, map_recording=False, going_save=False, 
               returning=False, initializing=False):
        """状態を更新"""
        self.tracking = tracking
        self.map_recording = map_recording
        self.going_save = going_save
        self.returning = returning
        self.initializing = initializing
        
        # 状態名を構築
        state_list = []
        if self.tracking:
            state_list.append("追跡状態")
        if self.map_recording:
            state_list.append("地図作成状態")
        if self.going_save:
            state_list.append("保存場所に向かう状態")
        if self.returning:
            state_list.append("戻って状態")
        if self.initializing:
            state_list.append("地図初期値に向かう状態")
        
        if not state_list:
            state_list.append("待機状態")
        
        self.current_state = " / ".join(state_list)
        return self.current_state

class LogManager:
    """ログ管理クラス"""
    def __init__(self):
        self.log_messages = []
    
    def add_log(self, message: str):
        """ログメッセージを追加"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_line = f"{timestamp} : {message}"
        print(log_line)
        self.log_messages.append(log_line)
    
    def get_logs(self):
        """ログメッセージを取得"""
        return self.log_messages.copy()

class WebApp:
    def __init__(self, processed_image_path: str):
        self.app = Flask(__name__)
        self.processed_image_path = processed_image_path
        self.robot_state = RobotState()
        self.log_manager = LogManager()
        
        # ログレベル設定
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.WARNING)
        
        # ルートを登録
        self._register_routes()
    
    def _register_routes(self):
        """Flaskルートを登録"""
        @self.app.route("/")
        def index():
            return render_template("index.html")
        
        @self.app.route("/captures/<path:filename>")
        def send_capture(filename):
            return send_from_directory(self.processed_image_path, filename)
        
        @self.app.route("/update")
        def update():
            # 追跡中の場合は最新キャプチャ、そうでなければ黒い画像を返す
            capture_url = self.get_latest_capture_url() if self.robot_state.tracking else "/captures/black.jpg"
            return jsonify(
                state=self.robot_state.current_state,
                logs=self.log_manager.get_logs(),
                capture=capture_url
            )
    
    def get_latest_capture_url(self) -> str:
        """最新キャプチャ画像のURLを取得"""
        image_files = glob.glob(os.path.join(self.processed_image_path, "detected_*.jpg"))
        if image_files:
            latest = max(image_files, key=os.path.getmtime)
            filename = os.path.basename(latest)
            return '/captures/' + filename
        return ""
    
    def log_event(self, message: str):
        """ログイベントを追加"""
        self.log_manager.add_log(message)
        # 状態更新ログの場合は状態も更新
        if message.startswith("状態更新:"):
            pass  # 状態は既に更新されている
    
    def update_robot_state(self, *, tracking=False, map_recording=False, 
                          going_save=False, returning=False, initializing=False):
        """ロボット状態を更新"""
        new_state = self.robot_state.update(
            tracking=tracking,
            map_recording=map_recording,
            going_save=going_save,
            returning=returning,
            initializing=initializing
        )
        self.log_event(f"状態更新: {new_state}")
    
    def run(self, host='0.0.0.0', port=5000, debug=False):
        """Webアプリケーションを実行"""
        self.app.run(host=host, port=port, debug=debug)
    
    def get_app(self) -> Flask:
        """Flaskアプリインスタンスを取得（テスト用）"""
        return self.app