# server_realsense_multiple について
複数のセンサPC から届く人の位置データーをまとめて、指定されたUnity クライアントに送る。

# 複数センサの構成、merge_realsense_multiple との関係
構成上、[merge_realsense_multiple](https://github.com/Placeholder-jp/CameraVision/tree/master/merge_realsense_multiple) は複数センサ用PC に入っていて、
server_realsense_multiple に検出された人の位置データーを転送するのみ。
データーマージはまとめてserver_realsense_multiple で行うので今後タイムスタンプ同期など細かいデーター操作するときを考えると 
server_realsense_multiple は（センサ）マスターとして機能するので __server__ が付く。

# proto ファイルについて
```/proto/``` フォルダ内にある proto ファイルは通信するデーター構造の定義を含みます。
この定義は、センサモジュール側（Client、Server両方）とUnityクライアント側すべてに存在しないといけない。
Proto ファイルやProtocol Buffer の詳細は[こちらのページ](https://developers.google.com/protocol-buffers/)をご参考ください。

# パラメーター
## 1個パラメーターがある場合
* Unity クライアントのIPアドレス。

## 8個パラメーターがある場合
2段階キャリブレーションを想定したパラメーターで、投影されるエリアのコーナー４つの位置情報。以下の順番で受け付ける。
* TopLeft_X
* TopLeft_Y
* TopRight_X
* TopRight_Y
* BottomLeft_X
* BottomLeft_Y
* BottomRight_X
* BottomRight_Y

## なにもパラメーターがない場合
デフォルト設定使用。Unity クライアントは同じローカルにあると設定される。

## 既知の制限
Unity クライアントのポートは __7777__ に固定されている。
