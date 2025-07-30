import time
import csv
import multiprocessing
import cv2
import os
from metavision_core.event_io import EventsIterator
from metavision_core.event_io.raw_reader import initiate_device
from metavision_hal import I_TriggerIn
from pypylon import pylon


# ========================
# Prophesee (Slave)
# ========================
def start_prophesee_slave(serial, barrier, stop_event):
    device = initiate_device(serial)
    print("[Prophesee] Device opened")

    # Enable Trigger In
    i_trigger_in = device.get_i_trigger_in()
    if i_trigger_in:
        i_trigger_in.enable(I_TriggerIn.Channel.MAIN)
        print("[Prophesee] Trigger In enabled")

    stream = device.get_i_events_stream()
    mv_iterator = EventsIterator.from_device(device=device)

    raw_filename = "Output/output_prophesee_12.raw"
    stream.log_raw_data(raw_filename)
    print(f"[Prophesee] Logging to {raw_filename}")

    barrier.wait()

    for evs in mv_iterator:
        if stop_event.is_set():
            break

    stream.stop_log_raw_data()
    print(f"[Prophesee] Saved to {raw_filename}")

# ========================
# Basler (Master)
# ========================
def start_basler_master(barrier, stop_event, duration):
    tl_factory = pylon.TlFactory.GetInstance()
    devices = tl_factory.EnumerateDevices()
    if not devices:
        print("[Basler] No camera detected.")
        return

    camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
    camera.Open()
    print("[Basler] Camera opened")

    # === Enable chunk mode and timestamp ===
    camera.ChunkModeActive = True
    camera.ChunkSelector = "Timestamp"
    camera.ChunkEnable = True
    print("[Basler] Chunk Timestamp enabled")

    # === Set Acquisition Frame Rate ===
    try:
        camera.AcquisitionFrameRateEnable = True
        camera.AcquisitionFrameRate.SetValue(169.0)
        print("[Basler] Acquisition Frame Rate set to 169 fps")
    except Exception as e:
        print(f"[Basler] Could not set Acquisition Frame Rate: {e}")

    # === Configure Line2 for VSync ===
    camera.LineSelector = "Line2"
    camera.LineMode = "Output"
    camera.LineSource = "ExposureActive"
    print("[Basler] Line2 configured for VSync output")

    tick_frequency = 1e9  # 1 ns ticks

    barrier.wait()

    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    print("[Basler] Grabbing frames to RAM...")

    frames_buffer = []
    frame_count = 0
    t_start = time.time()

    while camera.IsGrabbing():
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        if grabResult.GrabSucceeded():
            img = grabResult.Array
            chunk_timestamp = grabResult.ChunkTimestamp.Value
            timestamp_sec = chunk_timestamp / tick_frequency

            frames_buffer.append((frame_count, img, chunk_timestamp, timestamp_sec))
            frame_count += 1

        grabResult.Release()

        if time.time() - t_start >= duration:
            break

    camera.StopGrabbing()
    camera.Close()

    elapsed = time.time() - t_start
    actual_fps = frame_count / elapsed if elapsed > 0 else 0.0

    # === Save everything ===
    save_dir = "basler_frames_12"
    os.makedirs(save_dir, exist_ok=True)

    
    with open('basler_chunk_timestamps_12.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['FrameIndex', 'ChunkTimestamp (ticks)', 'Timestamp (s)'])
        for frame_index, img, chunk_timestamp, timestamp_sec in frames_buffer:
            img_color = cv2.cvtColor(img, cv2.COLOR_BAYER_RG2BGR)
            filename = os.path.join(save_dir, f"frame_{frame_index:05d}.tiff")
            cv2.imwrite(filename, img_color)
            writer.writerow([frame_index, chunk_timestamp, timestamp_sec])

    print(f"[Basler] Saved {frame_count} frames to '{save_dir}'")
    print(f"[Basler] Chunk timestamps saved to 'basler_chunk_timestamps_12.csv'")
    print(f"[Basler] Actual FPS: {actual_fps:.2f}")
    # === Signal Prophesee to stop ===
    stop_event.set()
    

# ========================
# Main startup
# ========================
if __name__ == "__main__":
    barrier = multiprocessing.Barrier(2)
    stop_event = multiprocessing.Event()
    duration = 2  # seconds

    serial_number = "00051849"

    p1 = multiprocessing.Process(target=start_prophesee_slave, args=(serial_number, barrier, stop_event))
    p2 = multiprocessing.Process(target=start_basler_master, args=(barrier, stop_event, duration))

    p1.start()
    p2.start()

    p2.join()  # Wait for Basler to finish
    p1.join()  # Then Prophesee stops once event is set

    print("✅ Synchronized acquisition complete.")