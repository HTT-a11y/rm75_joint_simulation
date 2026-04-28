import re
import os

# 1. 获取当前脚本所在的绝对路径（极其关键！）
script_dir = os.path.dirname(os.path.abspath(__file__))

# 2. 拼接出官方文件的真实绝对路径
filepath = os.path.join(script_dir, 'rm_75.urdf.xacro')
outpath = os.path.join(script_dir, 'rm_75_macro.urdf.xacro')

print(f"正在读取文件: {filepath}")

if not os.path.exists(filepath):
    print("错误：找不到原始文件！请确保你把这个 python 脚本放在了 src/ros2_rm_robot/rm_description/urdf/ 目录下！")
    exit(1)

with open(filepath, 'r', encoding='utf-8') as f:
    content = f.read()

# 3. 提取官方文件中所有的真实 link 和 joint 名称
links = re.findall(r'<link\s+name="([^"]+)"', content)
joints = re.findall(r'<joint\s+name="([^"]+)"', content)

links = [l for l in links if '$' not in l]
joints = [j for j in joints if '$' not in j]

# 4. 精准注入 prefix 前缀，保留所有官方物理数据
for l in set(links):
    content = content.replace(f'name="{l}"', f'name="${{prefix}}{l}"')
    content = content.replace(f'link="{l}"', f'link="${{prefix}}{l}"')

for j in set(joints):
    content = content.replace(f'name="{j}"', f'name="${{prefix}}{j}"')

# 5. 抹除可能会造成冲突的宏变量和属性
content = re.sub(r'<xacro:arg[^>]*/>', '', content)
content = re.sub(r'<xacro:property[^>]*/>', '', content)
content = content.replace('$(arg link7_type)', '${prefix}Link7')
content = content.replace('${link7_type_property}', '${prefix}Link7')

# 6. 去除原文件的 <robot> 外壳
content = re.sub(r'<\?xml.*?\?>', '', content)
content = re.sub(r'<robot[^>]*>', '', content)
content = content.replace('</robot>', '')

# 7. 重新包装为标准宏
macro_header = """<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="rm_75_robot" params="prefix parent *origin">
    <joint name="${prefix}base_joint" type="fixed">
      <xacro:insert_block name="origin" />
      <parent link="${parent}" />
      <child link="${prefix}base_link" />
    </joint>
"""
macro_footer = "\n  </xacro:macro>\n</robot>\n"

with open(outpath, 'w', encoding='utf-8') as f:
    f.write(macro_header + content + macro_footer)

print(f"✅ 完美生成！已保存至: {outpath}")
