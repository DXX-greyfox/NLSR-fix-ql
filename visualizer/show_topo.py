#!/usr/bin/env python3
import json
import sys
import os
from pathlib import Path
import re
import time
from flask import Flask, render_template, send_from_directory, abort, jsonify

# --- 配置 ---
DEFAULT_CONF_FILE = "/usr/local/etc/ndn/nlsr.conf"
DEFAULT_STATE_DIR = "/var/lib/nlsr"
TOPOLOGY_FILE_NAME = "topology.json"
POLL_TIMEOUT_SEC = 30  # 长轮询超时时间
POLL_INTERVAL_SEC = 0.5 # 检查文件变化的间隔

# 全局变量
topology_json_path = None
# 用于存储 topology.json 的最后修改时间
g_last_mod_time = 0.0 

# --- Flask Web 应用 ---
app = Flask(__name__,
            static_folder=os.path.abspath(os.path.dirname(__file__)),
            template_folder=os.path.abspath(os.path.dirname(__file__)))

def find_state_dir(conf_file_path):
    """
    手动解析 nlsr.conf 文件以查找 state-dir。
    """
    try:
        with open(conf_file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if ';' in line:
                    line = line.split(';', 1)[0].strip()
                
                match = re.match(r'^\s*state-dir\s+([^\s;]+)', line)
                if match:
                    state_dir = match.group(1).rstrip(';')
                    print(f"Info: 在 {conf_file_path} 中找到 state-dir: {state_dir}", file=sys.stderr)
                    return state_dir
                        
    except FileNotFoundError:
        print(f"Warning: 配置文件未找到于: {conf_file_path}", file=sys.stderr)
    except Exception as e:
        print(f"Warning: 读取配置文件 {conf_file_path} 时出错. 错误: {e}", file=sys.stderr)
    
    print(f"Info: 未在配置文件中找到 'state-dir'，使用默认值: {DEFAULT_STATE_DIR}", file=sys.stderr)
    return DEFAULT_STATE_DIR

def initialize_paths():
    """
    在服务器启动时初始化所有文件路径。
    """
    global topology_json_path, g_last_mod_time
    
    # 1. 查找 nlsr.conf
    conf_locations = [DEFAULT_CONF_FILE, './nlsr.conf']
    conf_path = os.environ.get('NLSR_CONF_PATH')
    if not conf_path:
        for loc in conf_locations:
            if os.path.exists(loc):
                conf_path = loc
                break
        if not conf_path:
            conf_path = DEFAULT_CONF_FILE 
            
    print(f"Info: 正在使用配置文件: {conf_path}", file=sys.stderr)
    
    # 2. 查找 state-dir
    state_dir = find_state_dir(conf_path)
    
    # 3. 确定 topology.json 的最终路径
    topology_json_path = Path(state_dir) / TOPOLOGY_FILE_NAME
    
    # 4. 初始化文件修改时间
    try:
        g_last_mod_time = os.path.getmtime(topology_json_path)
    except FileNotFoundError:
        g_last_mod_time = 0.0
        print(f"Warning: 拓扑文件尚未找到。", file=sys.stderr)
        print(f"         (路径: {topology_json_path})", file=sys.stderr)
        print(f"Info:    请确保NLSR (nlsr.service) 正在运行。脚本将等待文件创建...", file=sys.stderr)

# --- Web 服务器路由 ---

@app.route('/')
def index():
    """
    路由 1: 'http://127.0.0.1:8080/'
    提供主网页 (index.html)。
    """
    print("Info: 浏览器已连接，正在提供 index.html", file=sys.stderr)
    return render_template('index.html')

@app.route('/api/topology')
def get_topology_long_poll():
    """
    路由 2: 'http://127.0.0.1:8080/api/topology' (长轮询)
    监视 topology.json 文件的变化。
    """
    global g_last_mod_time
    
    start_time = time.time()
    
    while time.time() - start_time < POLL_TIMEOUT_SEC:
        try:
            # 检查文件的当前修改时间
            current_mod_time = os.path.getmtime(topology_json_path)
            
            # (关键逻辑) 如果文件时间戳已更新 (C++端已重写文件)
            if current_mod_time > g_last_mod_time:
                g_last_mod_time = current_mod_time
                
                # 读取并返回新数据
                with open(topology_json_path, 'r') as f:
                    data = f.read()
                
                print(f"Info: 检测到文件更新，正在向浏览器发送新拓扑。", file=sys.stderr)
                return app.response_class(
                    response=data,
                    status=200, # 200 OK
                    mimetype='application/json'
                )
        
        except FileNotFoundError:
            # 如果文件被删除了（例如NLSR重启了），也算是一种“变化”
            if g_last_mod_time != 0.0:
                print(f"Info: 检测到文件被删除，发送空拓扑。", file=sys.stderr)
                g_last_mod_time = 0.0
                return jsonify({"nodes": [], "links": []}) # 返回空拓扑
        
        except Exception as e:
            print(f"Error: /api/topology: 检查文件时出错: {e}", file=sys.stderr)
            return abort(500)
            
        # 暂停 0.5 秒再检查
        time.sleep(POLL_INTERVAL_SEC)
    
    # (超时) 在 POLL_TIMEOUT_SEC 秒内文件没有变化
    print(f"Info: 文件未变化，长轮询超时。", file=sys.stderr)
    return app.response_class(status=304) # 304 Not Modified

# --- 主程序入口 ---

if __name__ == "__main__":
    # 1. 确定所有路径
    initialize_paths()
    
    # 2. 启动服务器
    print("\n" + "="*50)
    print(f"  NLSR Web 可视化服务器已启动 (长轮询模式)")
    print(f"  在您的浏览器中打开: http://127.0.0.1:8080")
    print(f"  (按 CTRL+C 停止服务器)")
    print(f"  监控文件: {topology_json_path}")
    print("="*50 + "\n")
    
    try:
        # 运行 Flask 服务器，端口为 8080
        # 关闭Flask的调试和重载器，确保我们的长轮询逻辑正常工作
        app.run(host='127.0.0.1', port=8080, debug=False, use_reloader=False)
        
    except OSError as e:
        if "Address already in use" in str(e):
            print(f"Error: 端口 8080 已被占用。", file=sys.stderr)
            print("       请关闭占用该端口的程序，或修改 show_topo.py 中的 'port=8080' 行。", file=sys.stderr)
        else:
            print(f"Error: 启动服务器失败: {e}", file=sys.stderr)
    except Exception as e:
        print(f"发生意外错误: {e}", file=sys.stderr)