#/bin/bash
echo "xboxdrvのインストール"
sudo apt-get install xboxdrv



echo "-----------------------------------------------"
echo "PS3コントトーラををPCに挿して、PSボタンをおして有効にする"
echo "-----------------------------------------------"



sudo xboxdrv --detach-kernel-driver 

echo "-----------------------------------------------"
echo "コンソールに出力が出てきたら成功"
echo "-----------------------------------------------"
echo "/dev/input/js0 が出ていれば成功"
