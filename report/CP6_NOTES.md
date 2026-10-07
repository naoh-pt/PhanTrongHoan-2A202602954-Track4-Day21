# Gợi ý trình bày CP6 — 3 phút

Mở sẵn [REPORT.md](REPORT.md), [ảnh demo](../results/figures/obstacle_demo_000011.png),
[biểu đồ benchmark](../results/figures/obstacle_parameter_sweep.png) và
[ảnh failure](../results/figures/fail_01_merged_pedestrians.png).

| Thời gian | Nội dung cần nói |
|---|---|
| 0:00–0:30 | Bài toán: phát hiện vật cản gần từ LiDAR, không dùng deep learning. Claim chỉ áp dụng cho tổng số cụm gần trên ba frame KITTI đã chọn. |
| 0:30–1:10 | Chỉ ảnh demo: lọc ROI, voxel downsample, RANSAC tách đất, DBSCAN và AABB. Frame `000011`: 53.503 điểm ROI → 8.652 sau voxel → 34 cụm. |
| 1:10–2:00 | Chỉ biểu đồ: quét riêng hai tham số, seed 0. Voxel 0,10 → 0,40 m làm tổng cụm gần 72 → 57 (−20,8%); xu hướng từng frame không đều. Latency đo sau warm-up, 20 lần/cấu hình. |
| 2:00–2:40 | Chỉ ảnh failure: frame `000043` có 4 Pedestrian GT nhưng DBSCAN `eps=0,60 m` chỉ cho 2 cụm chứa tâm GT. Giảm `eps=0,40 m` được 3 cụm, vẫn gộp P1/P2. Lỗi chính: Preprocess; đếm cụm không phải đếm người. |
| 2:40–3:00 | Ứng dụng robot kho: chọn voxel theo đánh đổi tốc độ/số cụm; theo dõi cụm rộng, khoảng cách gần nhất, tỷ lệ điểm mặt đất và latency p95. Cần thử trên LiDAR robot và kiểm tra với GT trước khi dùng để tránh va chạm. |

## Câu hỏi dễ gặp

- **Claim có đúng cho mọi frame hoặc sensor không?** Không. Đây là tổng trên ba frame KITTI; frame riêng lẻ có xu hướng khác. nuScenes có hệ trục và mật độ LiDAR khác, cần sửa ROI theo hệ trục rồi chạy lại benchmark.
- **Vì sao số cụm giảm không đồng nghĩa phát hiện kém hơn?** Cụm có thể mất, bị gộp, hoặc nhiễu bị loại. Muốn kết luận recall phải so khớp từng vật thể với GT box.
- **RANSAC làm mất vật thấp thì sao?** Điểm của vật gần mặt đất có thể bị tính là inlier của mặt phẳng. Theo dõi tỷ lệ ground, kiểm tra các vật thấp có GT và dùng ngưỡng/ROI phù hợp cảm biến.
- **Latency đo gì?** Voxel, RANSAC, DBSCAN và tạo AABB trên CPU Intel i5-1035G1; không gồm đọc file và crop ROI. Mỗi cấu hình bỏ lần đầu, sau đó đo 20 lần để lấy p50/p95.
- **Vì sao hai người bị gộp?** DBSCAN nối các điểm cách nhau trong bán kính `eps`; hai người đứng sát nhau tạo một thành phần liên thông. Giảm `eps` chỉ tách được một cặp trong ví dụ này.
