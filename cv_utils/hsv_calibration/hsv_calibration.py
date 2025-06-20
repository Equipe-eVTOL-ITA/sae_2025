import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import depthai as dai
import cv2
import threading
import os
import numpy as np

class OAK_HSVCalibratorPublisher(Node):
    def __init__(self):
        super().__init__('oak_hsv_calibrator_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz
        self.bridge = CvBridge()

        # Initialize DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Define camera source
        self.cam_rgb = self.pipeline.createColorCamera()
        
        self.image_width = 800
        self.image_height = 800
        self.x1 = int((self.image_width/2) - 25)
        self.x2 = int((self.image_width/2) + 25)

        self.cam_rgb.setPreviewSize(self.image_width, self.image_height)
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

        # Link the RGB output to XLink
        self.xout_rgb = self.pipeline.createXLinkOut()
        self.xout_rgb.setStreamName("rgb")
        self.cam_rgb.preview.link(self.xout_rgb.input)

        # Start DepthAI device
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        self.latest_frame = None
        self.latest_frame_crosshair = None
        self.should_exit = False
        self.frame_lock = threading.Lock()

        # Cria pasta para salvar imagens, se não existir
        os.makedirs("./imagens_crosshair", exist_ok=True)

        # Inicia thread para monitorar input do terminal
        self.input_thread = threading.Thread(target=self.listen_terminal, daemon=True)
        self.input_thread.start()

        self.detected_colors = []

    def listen_terminal(self):
        print("\n=== HSV Calibração Tool ===")
        print("Instruções:")
        print("  - Pressione ENTER para capturar uma amostra de cor")
        print("  - Pressione ESPAÇO para finalizar e calcular as faixas HSV")
        print("  - Mire o retângulo vermelho na cor que deseja calibrar")
        print("  - Capture pelo menos 5-10 amostras para melhor precisão")
        print("  - Varie ligeiramente a posição/iluminação entre as amostras")
        print("================================\n")
        
        while not self.should_exit:
            user_input = input()
            if user_input == '':  # ENTER
                with self.frame_lock:
                    frame = self.latest_frame.copy() if self.latest_frame is not None else None
                    frame_crosshair = self.latest_frame_crosshair.copy() if self.latest_frame_crosshair is not None else None
                if frame is not None:
                    hsv_color, stddev_hsv = self.get_frame_hsv_range(frame)
                    
                    # Quality check: reject samples with too much variation
                    max_allowed_std = 100  # Maximum standard deviation for any HSV channel
                    if max(stddev_hsv) > max_allowed_std:
                        print(f"❌ Amostra rejeitada - muita variação na região (std: {stddev_hsv})")
                        print(f"   Tente uma região mais uniforme (std máximo permitido: {max_allowed_std})")
                        continue
                    
                    self.detected_colors.append(hsv_color)
                    print(f"✅ Amostra {len(self.detected_colors)} capturada: H={hsv_color[0]}, S={hsv_color[1]}, V={hsv_color[2]} (std: {stddev_hsv.round(1)})")
                    self.save_pic_with_crosshair(frame_crosshair)
                    
                    # Show progress
                    min_samples = 5
                    if len(self.detected_colors) < min_samples:
                        remaining = min_samples - len(self.detected_colors)
                        print(f"   📊 Capture mais {remaining} amostras para calibração mínima")
                    else:
                        print(f"   📊 {len(self.detected_colors)} amostras coletadas - pronto para calibrar!")
                        
                else:
                    print("❌ Nenhuma imagem disponível para capturar.")
                    
            elif user_input == ' ':
                if len(self.detected_colors) < 3:
                    print(f"❌ Muito poucas amostras ({len(self.detected_colors)}). Capture pelo menos 3 amostras antes de calibrar.")
                    continue
                    
                print(f"\n🔧 Finalizando calibração com {len(self.detected_colors)} amostras...")
                self.should_exit = True
                self.calibrate()

    def save_pic_with_crosshair(self, frame=None):
        if frame is None:
            with self.frame_lock:
                frame = self.latest_frame_crosshair.copy() if self.latest_frame_crosshair is not None else None
        if frame is not None:
            filename = f"./imagens_crosshair/crosshair_{rclpy.clock.Clock().now().nanoseconds}.png"
            cv2.imwrite(filename, frame)
            print(f"Imagem salva em {filename}")
        else:
            print("Nenhuma imagem disponível para salvar.")

    def get_frame_hsv_range(self, bgr_image):
        # Convert BGR to HSV directly for better color space accuracy
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        n_y, n_x = hsv_image.shape[:2]
        
        # Ensure boundaries don't exceed image dimensions
        x1 = max(0, min(self.x1, n_x-1))
        x2 = max(0, min(self.x2, n_x))
        
        # Extract the center rectangle region
        roi = hsv_image[:, x1:x2]
        
        # Calculate mean and standard deviation for the HSV region
        mean_hsv = cv2.mean(roi)[:3]  # Get H, S, V means
        _, stddev_hsv = cv2.meanStdDev(roi)
        stddev_hsv = stddev_hsv.flatten()[:3]  # Get H, S, V standard deviations
        
        return np.array(mean_hsv, dtype=np.uint8), stddev_hsv

    def calibrate(self):
        """
        Enhanced calibration method that calculates robust HSV ranges
        from collected color data points.
        """
        if not self.detected_colors:
            print("Nenhuma cor detectada para calibrar.")
            return
            
        # Convert to numpy array for easier calculations
        colors_array = np.array(self.detected_colors)
        
        print(f"\nCalibração iniciada com {len(self.detected_colors)} amostras de cor:")
        print(f"Dados coletados (H, S, V):")
        for i, color in enumerate(self.detected_colors):
            print(f"  Amostra {i+1}: {color}")
        
        # Calculate statistics for each HSV channel
        h_values = colors_array[:, 0]
        s_values = colors_array[:, 1]
        v_values = colors_array[:, 2]
        
        # Calculate mean and standard deviation
        h_mean, h_std = np.mean(h_values), np.std(h_values)
        s_mean, s_std = np.mean(s_values), np.std(s_values)
        v_mean, v_std = np.mean(v_values), np.std(v_values)
        
        print(f"\nEstatísticas calculadas:")
        print(f"  Hue (H):        μ={h_mean:.1f}, σ={h_std:.1f}")
        print(f"  Saturation (S): μ={s_mean:.1f}, σ={s_std:.1f}")
        print(f"  Value (V):      μ={v_mean:.1f}, σ={v_std:.1f}")
        
        # Create ranges using 2.5 standard deviations (covers ~99% of data)
        # This is more robust than just min/max which can be affected by outliers
        std_multiplier = 2.5
        
        h_lower = max(0, h_mean - std_multiplier * h_std)
        h_upper = min(179, h_mean + std_multiplier * h_std)
        
        s_lower = max(0, s_mean - std_multiplier * s_std)
        s_upper = min(255, s_mean + std_multiplier * s_std)
        
        v_lower = max(0, v_mean - std_multiplier * v_std)
        v_upper = min(255, v_mean + std_multiplier * v_std)
        
        # Ensure minimum useful ranges
        min_h_range = 15  # Minimum hue range of 15 degrees
        min_sv_range = 30  # Minimum saturation/value range
        
        if h_upper - h_lower < min_h_range:
            center = (h_upper + h_lower) / 2
            h_lower = max(0, center - min_h_range/2)
            h_upper = min(179, center + min_h_range/2)
            
        if s_upper - s_lower < min_sv_range:
            center = (s_upper + s_lower) / 2
            s_lower = max(0, center - min_sv_range/2)
            s_upper = min(255, center + min_sv_range/2)
            
        if v_upper - v_lower < min_sv_range:
            center = (v_upper + v_lower) / 2
            v_lower = max(0, center - min_sv_range/2)
            v_upper = min(255, center + min_sv_range/2)
        
        # Create final ranges
        lower_hsv = np.array([int(h_lower), int(s_lower), int(v_lower)])
        upper_hsv = np.array([int(h_upper), int(s_upper), int(v_upper)])
        
        print(f"\nFaixas HSV calculadas:")
        print(f"  Lower bound: [{lower_hsv[0]}, {lower_hsv[1]}, {lower_hsv[2]}]")
        print(f"  Upper bound: [{upper_hsv[0]}, {upper_hsv[1]}, {upper_hsv[2]}]")
        print(f"\nPara usar no código:")
        print(f"  lower_hsv = np.array({lower_hsv.tolist()})")
        print(f"  upper_hsv = np.array({upper_hsv.tolist()})")
        
        # Also show the simple min/max range for comparison
        simple_range = [np.min(colors_array, axis=0), np.max(colors_array, axis=0)]
        print(f"\nFaixa simples (min/max) para comparação:")
        print(f"  Min: {simple_range[0]}")
        print(f"  Max: {simple_range[1]}")
        
        # Save results to file
        self.save_calibration_results(lower_hsv, upper_hsv, colors_array)
        
        return lower_hsv, upper_hsv
    
    def save_calibration_results(self, lower_hsv, upper_hsv, samples):
        """Save calibration results to a text file for easy reference."""
        timestamp = rclpy.clock.Clock().now().nanoseconds
        filename = f"./hsv_calibration_results_{timestamp}.txt"
        
        with open(filename, 'w') as f:
            f.write("=== HSV Calibration Results ===\n\n")
            f.write(f"Timestamp: {timestamp}\n")
            f.write(f"Number of samples: {len(samples)}\n\n")
            
            f.write("Raw sample data (H, S, V):\n")
            for i, sample in enumerate(samples):
                f.write(f"  Sample {i+1}: [{sample[0]}, {sample[1]}, {sample[2]}]\n")
            
            f.write(f"\nCalculated HSV ranges:\n")
            f.write(f"  Lower bound: [{lower_hsv[0]}, {lower_hsv[1]}, {lower_hsv[2]}]\n")
            f.write(f"  Upper bound: [{upper_hsv[0]}, {upper_hsv[1]}, {upper_hsv[2]}]\n\n")
            
            f.write("Code snippets for use:\n")
            f.write("Python/OpenCV:\n")
            f.write(f"  lower_hsv = np.array({lower_hsv.tolist()})\n")
            f.write(f"  upper_hsv = np.array({upper_hsv.tolist()})\n")
            f.write(f"  mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)\n\n")
            
            f.write("YAML format:\n")
            f.write(f"  hsv_lower: [{lower_hsv[0]}, {lower_hsv[1]}, {lower_hsv[2]}]\n")
            f.write(f"  hsv_upper: [{upper_hsv[0]}, {upper_hsv[1]}, {upper_hsv[2]}]\n")
        
        print(f"\n📁 Resultados salvos em: {filename}")
        
        return lower_hsv, upper_hsv

    def timer_callback(self):
        in_rgb = self.q_rgb.tryGet()  # Non-blocking call to get the latest RGB frame
        if in_rgb:
            frame = in_rgb.getCvFrame()  # Convert to OpenCV format
            with self.frame_lock:
                self.latest_frame = frame.copy()
                
                # Draw sampling region with better visual feedback
                height = frame.shape[0]
                
                # Draw vertical lines for the sampling region
                cv2.line(frame, (int(self.x1), 0), (int(self.x1), height), (0, 0, 255), 2)
                cv2.line(frame, (int(self.x2), 0), (int(self.x2), height), (0, 0, 255), 2)
                
                # Draw horizontal lines to create a complete rectangle
                cv2.line(frame, (int(self.x1), 0), (int(self.x2), 0), (0, 0, 255), 2)
                cv2.line(frame, (int(self.x1), height-1), (int(self.x2), height-1), (0, 0, 255), 2)
                
                # Add text overlay with instructions and current status
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                font_thickness = 2
                
                # Background rectangle for text
                cv2.rectangle(frame, (10, 10), (250, 60), (0, 0, 0), -1)
                cv2.rectangle(frame, (10, 10), (250, 60), (255, 255, 255), 2)
                
                # Instructions text
                cv2.putText(frame, f"Amostras coletadas: {len(self.detected_colors)}", 
                           (15, 50), font, font_scale, (0, 255, 0), font_thickness)
                
                # Show current HSV values in the sampling region
                if hasattr(self, 'latest_frame') and self.latest_frame is not None:
                    try:
                        # Draw a solid black background behind the bottom HSV text
                        cv2.rectangle(frame, (10, height - 60), (300, height), (0, 0, 0), -1)
                        current_hsv, current_std = self.get_frame_hsv_range(self.latest_frame)
                        cv2.putText(frame, f"HSV atual: H={current_hsv[0]} S={current_hsv[1]} V={current_hsv[2]}", 
                                   (15, height - 40), font, 0.5, (255, 255, 255), 1)
                        cv2.putText(frame, f"Std: {current_std.round(1)}", 
                                   (15, height - 20), font, 0.5, (255, 255, 255), 1)
                    except:
                        pass  # Ignore errors during HSV calculation
                
                self.latest_frame_crosshair = frame.copy()
                
            # Convert the OpenCV frame to a compressed ROS 2 image message
            msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.publisher_.publish(msg)
            # Remove the frequent logging to reduce console spam
            # self.get_logger().info('Published image frame')


def main(args=None):
    rclpy.init(args=args)
    oak_hsv_calibrator_publisher = OAK_HSVCalibratorPublisher()
    rclpy.spin(oak_hsv_calibrator_publisher)
    oak_hsv_calibrator_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()