#!/bin/bash
# 一键提交并推送到 GitHub

# 获取提交信息，如果没有输入则默认 "update"
read -p "请输入提交信息（默认: update）: " msg
if [ -z "$msg" ]; then
    msg="update"
fi

# 添加所有文件
git add .

# 提交
git commit -m "$msg"

# 推送到 main 分支
git push origin main
