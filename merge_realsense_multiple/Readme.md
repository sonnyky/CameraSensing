# merge_realsense_multiple (Client) について
複数のリアルセンスを用いた物体検出。検出結果を指定されたIP アドレスに送る。

# 複数センサの構成、server_realsense_multiple との関係
構成上、merge_realsense_multiple は複数センサ用PC に入っていて、[server_realsense_multiple](https://github.com/Placeholder-jp/CameraVision/tree/master/server_realsense_multiple) に検出された人の位置データーを転送するのみ。データーマージはまとめてserver_realsense_multiple で行うので今後タイムスタンプ同期など細かいデーター操作するときを考えると merge_realsense_multiple は（センサ）クライアントとして機能するので __Client__ が付く。

# proto ファイルについて
```/proto/``` フォルダ内にある proto ファイルは通信するデーター構造の定義を含みます。この定義は、センサモジュール側とUnityクライアント側両方に存在しないといけない。Proto ファイルやProtocol Buffer の詳細は[こちらのページ](https://developers.google.com/protocol-buffers/)をご参考ください。

# パラメーター
- Server IP：検出結果の送信先。
- Near Min Value：（デバイスから見た）近い検出レンジの最小値
- Near Max Value：（デバイスから見た）近い検出レンジの最大値
- Far Min Value：（デバイスから見た）遠い検出レンジの最小値
- Far Max Value：（デバイスから見た）遠い検出レンジの最大値
- Min Blob Area：検出されるブロブ（塊り）の最小値
- Max Blob Area：検出されるブロブ（塊り）の最大値
- Erosion Size：Dilate（ホールフィリング）のカーネルサイズ。この数値が大きくなるほどブロブの穴がなくなりきれいな形になるが、やりすぎるとノイズが目立つ。
- Adjustment：検出されたブロブ（塊り）の中心位置をどれぐらい強く中央に引き付けるか。
- Adjustment mode：座標変換をセンサモジュール側でやるか（0）Unityクライアント 側に任せるか（1）。Unity 側に任せる場合、1280x720 の画像上の位置を（0,1）のレンジに正規化される。デフォルト設定値は（1）になっている。
