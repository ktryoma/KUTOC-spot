import os
import glob
import logging
import datetime
from typing import Optional
from src.command_process import get_web_commands_dict
from flask import Flask, jsonify, render_template, send_from_directory, request

class WebServer:
    """Webサーバーとログ管理を行うクラス"""
    
    def __init__(self, processed_image_path="image"):
        self.app = Flask(__name__)
        self.log_messages = []
        self.current_state = "待機状態"
        self.processed_image_path = processed_image_path
        self.cp = None  # CommandProcessのインスタンスを保持
        self.command_dict = None
        self.lease = None  # LeaseClientのインスタンスを保持
        
        # 状態管理用のフラグ
        self.tracking_state = False          # 追跡状態
        self.map_recording_state = False     # 地図作成状態
        self.going_save_state = False        # 保存場所に向かう状態
        self.returning_state = False         # 戻って状態
        self.initializing_state = False      # 地図初期値に向かう状態
        
        # 不要なコンソール出力を減らすため、werkzeugのログレベルをWARNINGに設定
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.WARNING)
        
        # ルートの設定
        self._setup_routes()

    def set_cp(self, cp):
        self.cp = cp
        self.command_dict = get_web_commands_dict(self.cp)

    def set_lease(self, lease):
        self.lease = lease

    def _setup_routes(self):
        """Flaskのルートを設定"""
        
        @self.app.route("/captures/<path:filename>")
        def send_capture(filename):
            return send_from_directory(self.processed_image_path, filename)

        @self.app.route("/")
        def index():
            return render_template("index.html")

        @self.app.route("/update")
        def update():
            # 追跡中の場合は最新キャプチャ、そうでなければ黒い画像を返す
            capture_url = self.get_latest_capture_url() if self.tracking_state else "/captures/black.jpg"
            return jsonify(state=self.current_state, logs=self.log_messages, capture=capture_url)
        
        @self.app.route("/command", methods=["POST"])
        def command():
            """WebUIからのコマンドを処理"""
            try:
                data = request.get_json()
                command = data.get('command')
                
                if not self.cp:
                    return jsonify({"status": "error", "message": "Command processor not available"})
                
                if not self.command_dict:
                    return jsonify({"status": "error", "message": "Command dictionary not available"})
                
                # CommandProcessの関数を呼び出し
                result = self._execute_web_command(command)
                return jsonify({"status": "success", "message": result})
                
            except Exception as e:
                return jsonify({"status": "error", "message": str(e)})
        
        @self.app.route("/getlease", methods=["POST"])
        def get_lease():
            """操作権取得のエンドポイント"""
            try:
                # 操作権取得のロジックをここに実装
                if self.lease is not None:
                    print("lease is not none")
                    lease = self.lease.take()
                else:
                    print("lease is none")
                    return jsonify({"message": "操作権を取得できませんでした"})
                return jsonify({"message": "操作権を取得しました"})
            except Exception as e:
                return jsonify({"message": f"エラー: {str(e)}"})
    
    def _execute_web_command(self, command):
        """WebUIからのコマンドをCommandProcessに転送"""
        if not self.cp:
            return "Command processor not available"
        
        if not self.command_dict:
            return "Command dictionary not available"
        
        # get_web_commands_dict で定義されているコマンドマッピングを使用
        if command in self.command_dict:
            try:
                # コマンドを非同期実行（UIをブロックしないため）
                import threading
                command_func = self.command_dict[command]
                thread = threading.Thread(target=command_func, daemon=True)
                thread.start()
                self.log_event(f"Webコマンド実行: {command}")
                return f"コマンド '{command}' を実行しました"
            except Exception as e:
                self.log_event(f"Webコマンドエラー: {command} - {str(e)}")
                return f"コマンド実行エラー: {str(e)}"
        else:
            self.log_event(f"未知のWebコマンド: {command}")
            return f"未知のコマンド: {command}"
    
    
    def get_latest_capture_url(self):
        """最新キャプチャ画像のURLを取得するヘルパー関数"""
        import glob
        image_files = glob.glob(os.path.join(self.processed_image_path, "detected_*.jpg"))
        if image_files:
            latest = max(image_files, key=os.path.getmtime)
            filename = os.path.basename(latest)
            return '/captures/' + filename
        return ""
    
    def log_event(self, message):
        """イベントをログに記録"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_line = f"{timestamp} : {message}"
        print(log_line)
        self.log_messages.append(log_line)
    
    def update_robot_state(self, *, tracking=False, map_recording=False, going_save=False, returning=False, initializing=False):
        """
        各状態のフラグを引数として受け取り、複数の状態がTrueのときは全ての状態名を連結して表示します。
        すべてFalseの場合は「待機状態」として更新します。
        """
        # 各状態の更新
        self.tracking_state = tracking
        self.map_recording_state = map_recording
        self.going_save_state = going_save
        self.returning_state = returning
        self.initializing_state = initializing

        # 各状態がTrueの場合、その状態名をリストに追加
        state_list = []
        if self.tracking_state:
            state_list.append("追跡状態")
        if self.map_recording_state:
            state_list.append("地図作成状態")
        if self.going_save_state:
            state_list.append("保存場所に向かう状態")
        if self.returning_state:
            state_list.append("戻って状態")
        if self.initializing_state:
            state_list.append("地図初期値に向かう状態")
        # いずれもTrueでなければ「待機状態」とする
        if not state_list:
            state_list.append("待機状態")
        new_state = " / ".join(state_list)
        self.current_state = new_state
        self.log_event(f"状態更新: {new_state}")