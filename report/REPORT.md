# Báo cáo Day 6: Ảnh hưởng của voxel downsample và ngưỡng tách mặt đất đến phát hiện vật cản gần bằng LiDAR

> Thay **mọi** ô có chữ ĐIỀN nằm trong ngoặc vuông bằng nội dung của bạn, xoá luôn cả dấu ngoặc vuông. Lệnh `python tools/check_submission.py` sẽ báo FAIL nếu còn sót bất kỳ chỗ nào.

- **Họ tên:** Phan Trọng Hoàn
- **MSSV:** 2A202602954
- **Lớp:** Track 4
- **Link repo:** https://github.com/naoh-pt/PhanTrongHoan-2A202602954-Track4-Day21
- **Topic:** D — Robot/drone obstacle
- **Dataset:** `data/kitti_mini` (thí nghiệm chính); `data/synthetic` (kiểm tra pipeline)
- **Các frame chọn cho thí nghiệm:** `000011`, `000043`, `000049`

> Hãy viết ngắn: mỗi mục từ 3 đến 8 dòng, ưu tiên số liệu và hình ảnh.

## 1. Claim

Một câu khẳng định kỹ thuật có thể kiểm chứng. Ví dụ: *"Lệch yaw 1° làm 12% điểm LiDAR rơi ra khỏi vật thể ở 30 m, phát hiện được bằng edge-alignment score với ngưỡng X."*

**Giả thuyết CP1:** Trên các frame KITTI `000011`, `000043`, `000049`, trong vùng phía trước 0–30 m, tăng `voxel_size` từ 0,10 m lên 0,40 m sẽ làm giảm ít nhất 20% số cụm vật cản ở khoảng cách không quá 20 m, khi giữ ngưỡng tách đất và các tham số DBSCAN cố định. Để đạt mức Good của topic D, sẽ quét riêng ngưỡng RANSAC ở 0,10/0,20/0,30 m và đo số cụm, kích thước bounding box, khoảng cách tới cụm gần nhất cùng thời gian chạy.

## 2. Evidence

Bảng hoặc plot số liệu, kèm ảnh/video demo. Ghi rõ đường dẫn file trong `results/`.

| Cấu hình / mức perturb | Metric 1 | Metric 2 | Ghi chú |
|---|---|---|---|
| [ĐIỀN] | | | |

Demo CP2 trên KITTI `000011`: 53.503 điểm trong vùng quan tâm, còn 8.652 điểm sau voxel 0,20 m; RANSAC tách 4.540 điểm mặt đất và DBSCAN tạo 34 cụm. Đây là kiểm tra pipeline, chưa phải kết quả benchmark CP3.

![Demo bốn bước của pipeline phát hiện vật cản](../results/figures/obstacle_demo_000011.png)

## 3. Failure case

Nêu khi nào hệ thống hoặc phương pháp fail, vì sao fail, và liên hệ tới lớp nào trong 6 lớp debug: I/O, Geometry, Time, Preprocess, Model, Metric.

![failure](../results/figures/fail_[ĐIỀN].png)

[ĐIỀN]

## 4. Khuyến nghị nếu triển khai thật

Use-case cụ thể (ADAS / robot / drone), trade-off và bước tiếp theo.

[ĐIỀN]

## 5. Cách chạy lại

Từ gốc repo trên Windows PowerShell, chạy các lệnh sau để tái tạo demo CP2. Lệnh cho benchmark sẽ được bổ sung ở CP3.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m pip install "open3d>=0.18"
.\.venv\Scripts\python.exe -m src.obstacle_demo --data-root data/kitti_mini --frame 000011 --voxel-size 0.20 --ground-threshold 0.20 --eps 0.60 --min-points 8 --seed 0 --out results/figures/obstacle_demo_000011.png
```

## 6. Khai báo sử dụng AI

Ghi rõ đã dùng công cụ AI nào, dùng vào việc gì, và bạn đã tự kiểm chứng kết quả đó bằng cách nào. Nếu không dùng AI, ghi "Không sử dụng". Xem quy định ở `RULES.md` mục 2.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| OpenAI Codex | Hỗ trợ xây dựng script demo CP2 và rà soát báo cáo | Đã chạy trên KITTI `000011` và synthetic `000000`, kiểm tra số điểm, pháp tuyến mặt đất và ảnh BEV; học viên cần tự chạy và giải thích lại trước khi nộp |
