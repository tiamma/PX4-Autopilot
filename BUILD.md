## 配置板子
make aet_h743-basic_default boardconfigmake aet_h743-basic_default boardconfig

## 构建
make aet_h743-basic_default upload

i2cdetect -b 2

make cuav_fmu-v6x_default boardconfig
git config --global http.proxy http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
git fetch

git submodule update --init --recursive

git remote -v
git config --global --get-regexp "http.*proxy"

git config --global --unset http.proxy
git config --global --unset https.proxy


make cuav_fmu-v6x_default

wget -e "http_proxy=socks5://127.0.0.1:7897" "https://github.com/microsoft/onnxruntime/releases/download/v1.12.1/onnxruntime-win-x64-gpu-1.12.1.zip"


curl -x http://127.0.0.1:7897 -LO https://github.com/microsoft/onnxruntime/releases/download/v1.12.1/onnxruntime-win-x64-gpu-1.12.1.zip

curl -x http://127.0.0.1:7897 -LO https://raw.githubusercontent.com/nlohmann/json/v3.11.3/single_include/nlohmann/json.hpp
