"""診療所内の各地点（地図 = ワールド座標、単位 m / rad）"""
import math

# 名前: (x, y, 到着時の向き yaw)
STATIONS = {
    'reception': (-2.5, 0.0, 0.0),            # 受付（青）
    'exam':      (0.4, 1.9, math.pi / 2),     # 診察室（緑）
    'treatment': (2.9, 1.9, math.pi / 2),     # 処置室（橙）
    'lab':       (1.4, -1.9, -math.pi / 2),   # 検査室（赤）
}

LABELS = {
    'reception': '受付',
    'exam': '診察室',
    'treatment': '処置室',
    'lab': '検査室',
}
