## 配置板子
make aet_h743-basic_default boardconfig

## 构建
make aet_h743-basic_default upload

git config --global http.proxy http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
git fetch


git submodule update --init --recursive


git remote -v
git config --global --get-regexp "http.*proxy"

git config --global --unset http.proxy
git config --global --unset https.proxy
