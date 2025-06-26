import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import seaborn as sns
from io import StringIO

# 한글 폰트 설정 (matplotlib)
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

def analyze_drone_data(csv_data, target_fall_distance=None):
    """
    드론 비행 데이터를 분석하는 함수
    """
    # CSV 데이터 읽기
    if isinstance(csv_data, str):
        df = pd.read_csv(StringIO(csv_data))
    else:
        df = csv_data
    
    # 데이터 정리
    df.columns = df.columns.str.strip()  # 공백 제거
    
    # 모터 상태 판별
    #   • motor_output_avg == 0 → OFF
    #   • motor_output_avg != 0 → ON
    df['motor_on'] = df['motor_output_avg'] != 0
    
    # 물리 모델 상수
    A_DOWN = 9.4   # 자유 낙하 등가속도 (m/s²)
    A_UP   = 3.9   # 상승 등가속도 (m/s²)
    
    # 모터 상태 변화점 찾기
    motor_changes = df['motor_on'].diff().fillna(0)
    motor_off_points = df[motor_changes == -1].index.tolist()
    motor_on_points = df[motor_changes == 1].index.tolist()
    
    print("=== 드론 비행 데이터 분석 결과 ===\n")
    
    # 기본 통계
    print("1. 기본 비행 정보:")
    print(f"   - 총 비행 시간: {df['time'].iloc[-1] - df['time'].iloc[0]:.3f}초")
    print(f"   - 최고 고도: {df['altitude'].max():.3f}m")
    print(f"   - 최저 고도: {df['altitude'].min():.3f}m")
    print(f"   - 최대 상승 속도: {df['velocity_z'].max():.3f}m/s")
    print(f"   - 최대 하강 속도: {df['velocity_z'].min():.3f}m/s")
    print(f"   - 최대 상승 가속도: {df['acceleration_z'].max():.3f}m/s²")
    print(f"   - 최대 하강 가속도: {df['acceleration_z'].min():.3f}m/s²")
    
    # 모터 상태 분석
    print(f"\n2. 모터 동작 분석:")
    print(f"   - 모터 꺼진 구간 수: {len(motor_off_points)}")
    print(f"   - 모터 켜진 구간 수: {len(motor_on_points)}")
    
    # 추락 거리 정확한 계산 (가속도와 모터출력 기준)
    total_fall_distance = 0
    total_fall_time = 0
    fall_segments = []
    
    # 모터가 꺼진 모든 구간 마스크 (motor_output_avg == 0)
    motor_off_mask = df['motor_output_avg'] == 0
    
    # 연속된 모터 꺼짐 구간 찾기
    in_fall = False
    fall_start_idx = None
    
    for i in range(len(df)):
        if motor_off_mask.iloc[i] and not in_fall:
            # 추락 시작 (모터 꺼지고 중력가속도 감지)
            in_fall = True
            fall_start_idx = i
        elif not motor_off_mask.iloc[i] and in_fall:
            # 추락 끝 (모터 켜지거나 가속도 변화)
            in_fall = False
            fall_end_idx = i - 1
            
            # 추락 구간 데이터 계산
            fall_data = df.iloc[fall_start_idx:fall_end_idx+1]
            if len(fall_data) > 1:
                start_altitude = fall_data['altitude'].iloc[0]
                end_altitude = fall_data['altitude'].iloc[-1]
                fall_distance = start_altitude - end_altitude  # 양수면 추락
                fall_time = fall_data['time'].iloc[-1] - fall_data['time'].iloc[0]
                
                # 실제로 추락한 경우만 (고도가 감소한 경우)
                if fall_distance > 0.01:  # 1cm 이상 추락한 경우만
                    total_fall_distance += fall_distance
                    total_fall_time += fall_time
                    
                    # 추락 구간의 평균 가속도와 모터 출력
                    avg_accel = fall_data['acceleration_z'].mean()
                    avg_motor = fall_data['motor_output_avg'].mean()
                    
                    fall_segments.append({
                        'segment': len(fall_segments) + 1,
                        'start_time': fall_data['time'].iloc[0],
                        'end_time': fall_data['time'].iloc[-1],
                        'duration': fall_time,
                        'start_altitude': start_altitude,
                        'end_altitude': end_altitude,
                        'fall_distance': fall_distance,
                        'avg_fall_velocity': fall_distance / fall_time if fall_time > 0 else 0,
                        'avg_acceleration': avg_accel,
                        'avg_motor_output': avg_motor
                    })
    
    # 마지막까지 모터가 꺼져있는 경우 처리 (이전 세그먼트 기반 로직, 유지)
    if in_fall and fall_start_idx is not None:
        fall_data = df.iloc[fall_start_idx:]
        if len(fall_data) > 1:
            start_altitude = fall_data['altitude'].iloc[0]
            end_altitude = fall_data['altitude'].iloc[-1]
            fall_distance = start_altitude - end_altitude
            fall_time = fall_data['time'].iloc[-1] - fall_data['time'].iloc[0]
            
            if fall_distance > 0.01:
                total_fall_distance += fall_distance
                total_fall_time += fall_time
                
                avg_accel = fall_data['acceleration_z'].mean()
                avg_motor = fall_data['motor_output_avg'].mean()
                
                fall_segments.append({
                    'segment': len(fall_segments) + 1,
                    'start_time': fall_data['time'].iloc[0],
                    'end_time': fall_data['time'].iloc[-1],
                    'duration': fall_time,
                    'start_altitude': start_altitude,
                    'end_altitude': end_altitude,
                    'fall_distance': fall_distance,
                    'avg_fall_velocity': fall_distance / fall_time if fall_time > 0 else 0,
                    'avg_acceleration': avg_accel,
                    'avg_motor_output': avg_motor
                })
    
    # === 최고/최저 고도 기반 분석으로 재계산 ===
    altitude_max = df['altitude'].max()
    altitude_min = df['altitude'].min()
    total_fall_distance = altitude_max - altitude_min

    # 실제 모터 OFF 시간 (첫 OFF~ON 구간)
    real_off_time = 0.0
    if motor_off_points:
        off_start_idx = motor_off_points[0]
        if motor_on_points:
            on_start_idx = motor_on_points[0]
        else:
            on_start_idx = df.index[-1]
        real_off_time = df['time'].iloc[on_start_idx] - df['time'].iloc[off_start_idx]
        total_fall_time = real_off_time  # 총 추락 시간도 동일하게 설정

    print(f"\n3. 추락 분석 (최고-최저 고도 기준):")
    print(f"   - 총 추락 거리: {total_fall_distance:.3f}m")
    print(f"   - 모터 OFF 실제 시간: {real_off_time:.3f}초")
    
    # 목표 거리 달성을 위한 물리 모델 기반 필요 시간 계산
    if target_fall_distance is not None:
        print(f"\n4. 목표 거리 달성 분석 (물리 모델):")
        # (1/2) * a * t² = s  →  t = sqrt(2s/a)
        required_off_time = np.sqrt(2 * target_fall_distance / A_DOWN)
        required_on_time  = np.sqrt(2 * target_fall_distance / A_UP)
        print(f"   - 목표 추락 거리(고도 차이): {target_fall_distance:.3f}m")
        print(f"   - 필요 모터 OFF 시간(추락): {required_off_time:.3f}초 (a={A_DOWN}m/s²)")
        print(f"   - 필요 모터 ON  시간(상승): {required_on_time:.3f}초 (a={A_UP}m/s²)")
        if real_off_time > 0:
            print(f"   - 실제 모터 OFF 시간 대비 비율: {required_off_time/real_off_time:.2f}배")

    # 전체 고도 범위 출력 (최고-최저)
    altitude_range = df['altitude'].max() - df['altitude'].min()
    print(f"\n5. 전체 고도 변화량: {altitude_range:.3f}m")
    
    return df, fall_segments, total_fall_distance, total_fall_time

def plot_drone_data(df, fall_segments):
    """
    드론 데이터를 시각화하는 함수
    """
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('Drone Flight Data Analysis', fontsize=16, fontweight='bold')
    
    # 1. 고도 vs 시간 (모터 상태 표시)
    ax1 = axes[0, 0]
    ax1.plot(df['time'], df['altitude'], 'b-', linewidth=2, label='Altitude')
    
    # 모터 꺼진 구간 하이라이트 (motor_output_avg == 0)
    motor_off = df['motor_output_avg'] == 0
    for i in range(len(df)):
        if motor_off.iloc[i]:
            ax1.axvspan(df['time'].iloc[i], df['time'].iloc[min(i+1, len(df)-1)], 
                       alpha=0.3, color='red', label='Motor Off (Free Fall)' if i == motor_off.idxmax() else '')
    
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Altitude (m)')
    ax1.set_title('Altitude vs Time')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # 2. 속도 vs 시간
    ax2 = axes[0, 1]
    ax2.plot(df['time'], df['velocity_z'], 'g-', linewidth=2, label='Vertical Velocity')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity Z (m/s)')
    ax2.set_title('Vertical Velocity vs Time')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # 3. 가속도 vs 시간
    ax3 = axes[1, 0]
    ax3.plot(df['time'], df['acceleration_z'], 'r-', linewidth=2, label='Vertical Acceleration')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax3.axhline(y=-9.81, color='orange', linestyle=':', alpha=0.7, label='Gravity (-9.81 m/s²)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration Z (m/s²)')
    ax3.set_title('Vertical Acceleration vs Time')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # 4. 모터 출력 vs 시간 (가속도와 함께 표시)
    ax4 = axes[1, 1]
    
    # 모터 출력
    ax4_twin = ax4.twinx()
    line1 = ax4.plot(df['time'], df['motor_output_avg'], 'purple', linewidth=2, label='Motor Output')
    ax4.axhline(y=0.0, color='purple', linestyle='--', alpha=0.7, label='Motor OFF Threshold')
    
    # 가속도 (오른쪽 축)
    line2 = ax4_twin.plot(df['time'], df['acceleration_z'], 'orange', linewidth=2, label='Acceleration Z')
    ax4_twin.axhline(y=8.0, color='orange', linestyle='--', alpha=0.7, label='Gravity Threshold')
    ax4_twin.axhline(y=9.81, color='red', linestyle=':', alpha=0.7, label='Standard Gravity')
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Motor Output', color='purple')
    ax4_twin.set_ylabel('Acceleration Z (m/s²)', color='orange')
    ax4.set_title('Motor Output & Acceleration vs Time')
    ax4.grid(True, alpha=0.3)
    
    # 범례 합치기
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax4.legend(lines, labels, loc='upper right')
    
    plt.tight_layout()
    plt.show()
    
    # 추락 구간 상세 분석 플롯
    if fall_segments:
        fig2, ax = plt.subplots(figsize=(12, 8))
        
        # 전체 고도 플롯
        ax.plot(df['time'], df['altitude'], 'b-', linewidth=1, alpha=0.7, label='Altitude')
        
        # 각 추락 구간 하이라이트
        colors = ['red', 'orange', 'purple', 'brown', 'pink']
        for i, segment in enumerate(fall_segments):
            start_idx = df[df['time'] >= segment['start_time']].index[0]
            end_idx = df[df['time'] <= segment['end_time']].index[-1]
            
            segment_data = df.iloc[start_idx:end_idx+1]
            color = colors[i % len(colors)]
            
            ax.plot(segment_data['time'], segment_data['altitude'], 
                   color=color, linewidth=3, 
                   label=f'Fall Segment {segment["segment"]} ({segment["fall_distance"]:.2f}m)')
            
            # 시작점과 끝점 마크
            ax.scatter(segment['start_time'], segment['start_altitude'], 
                      color=color, s=100, marker='o', edgecolor='black', linewidth=2)
            ax.scatter(segment['end_time'], segment['end_altitude'], 
                      color=color, s=100, marker='s', edgecolor='black', linewidth=2)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Altitude (m)')
        ax.set_title('Fall Segments Analysis')
        ax.grid(True, alpha=0.3)
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        plt.tight_layout()
        plt.show()

# 예제 사용법
if __name__ == "__main__":
    # 제공된 CSV 데이터
    sample_data = """time,altitude,velocity_z,acceleration_z,motor_output_avg,control_mode,thrust_factor
0.062411,260.9384,0.000292,0.003125,0.5,UNKNOWN,0
0.174686,260.9386,3.65E-05,-0.00466,0.5,UNKNOWN,0
0.285892,260.9387,-5.81E-05,0.010855,0.5,UNKNOWN,0
0.399691,260.9388,0.000112,0.012769,0.5,UNKNOWN,0
0.507401,260.9388,0.000763,0.008493,0.5,UNKNOWN,0"""
    
    # 실제 사용 예제 (파일에서 읽기)
    target_distance = 19.0  # 원하는 추락 거리 (미터)
    df = pd.read_csv('/home/user1/ASP_Workspace/asp_ros2_ws/calibration_results/20250627_000057_123.csv')
    analyzed_df, fall_segments, total_fall_dist, total_fall_time = analyze_drone_data(df, target_distance)
    plot_drone_data(analyzed_df, fall_segments)
    
    # 샘플 데이터로 테스트
    # print("샘플 데이터 분석 (실제 데이터는 파일에서 읽어서 사용하세요):")
    # target_distance = 2.0  # 예시: 2미터 추락을 원할 때
    # analyzed_df, fall_segments, total_fall_dist, total_fall_time = analyze_drone_data(sample_data, target_distance)
    # plot_drone_data(analyzed_df, fall_segments)