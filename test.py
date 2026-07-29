from fsc_protocol import FSCController, WaveType
import time

with FSCController('COM10', 1) as fsc:
    # 切换闭环 + 发送位移
    fsc.set_loop_mode(channel=0, closed=True)
    fsc.set_loop_mode(channel=1, closed=True)
    fsc.set_signal_mode(channel=0, analog=False)
    fsc.set_signal_mode(channel=1, analog=False)
    fsc.send_displacement(channel=0, displacement=5)
    fsc.send_displacement(channel=1, displacement=8)
    # 读取当前位移
    time.sleep(1)
    d = fsc.read_displacement(channel=0)
    print(f"当前位移: {d} mrad")
    d = fsc.read_displacement(channel=1)
    print(f"当前位移: {d} mrad")
    # 发正弦波形（峰峰值10V，频率5Hz，偏置5V）
    time.sleep(1)
    # fsc.stop_wave(channel=0)
    # fsc.stop_wave(channel=1)
    print("发送正弦波形")
    print("频率20Hz，峰峰值0.65V，偏置5V")
    fsc.send_wave_displacement(
        channel=0,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=20,
        offset=5,
    )

    fsc.send_wave_displacement(
        channel=1,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=20,
        offset=8,
    )

    time.sleep(5)

    fsc.send_displacement(channel=0, displacement=5)
    fsc.send_displacement(channel=1, displacement=8)

    print("发送正弦波形")
    print("频率80Hz，峰峰值0.65V，偏置5V")
    fsc.send_wave_displacement(
        channel=0,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=80,
        offset=5,
    )

    fsc.send_wave_displacement(
        channel=1,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=80,
        offset=8,
    )

    time.sleep(5)

    fsc.send_displacement(channel=0, displacement=5)
    fsc.send_displacement(channel=1, displacement=8)

    fsc.send_displacement(channel=0, displacement=5)
    fsc.send_displacement(channel=1, displacement=8)

    print("发送正弦波形")
    print("频率120Hz，峰峰值0.65V，偏置5V")
    fsc.send_wave_displacement(
        channel=0,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=120,
        offset=5,
    )

    fsc.send_wave_displacement(
        channel=1,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=120,
        offset=8,
    )

    time.sleep(5)

    fsc.send_displacement(channel=0, displacement=5)
    fsc.send_displacement(channel=1, displacement=8)

    fsc.send_displacement(channel=0, displacement=5)
    fsc.send_displacement(channel=1, displacement=8)

    print("发送正弦波形")
    print("频率140Hz，峰峰值0.65V，偏置5V")
    fsc.send_wave_displacement(
        channel=0,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=140,
        offset=5,
    )

    fsc.send_wave_displacement(
        channel=1,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=140,
        offset=8,
    )

    time.sleep(5)

    fsc.send_displacement(channel=0, displacement=5)
    fsc.send_displacement(channel=1, displacement=8)

    fsc.send_displacement(channel=0, displacement=5)
    fsc.send_displacement(channel=1, displacement=8)

    print("发送正弦波形")
    print("频率160Hz，峰峰值0.65V，偏置5V")
    fsc.send_wave_displacement(
        channel=0,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=160,
        offset=5,
    )

    fsc.send_wave_displacement(
        channel=1,
        wave_type=WaveType.SINE,
        peak_peak=0.65,
        frequency=160,
        offset=8,
    )

    time.sleep(5)

    fsc.send_displacement(channel=0, displacement=5)
    fsc.send_displacement(channel=1, displacement=8)
