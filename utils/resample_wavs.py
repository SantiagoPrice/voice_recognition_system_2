import argparse
from pathlib import Path
import scipy.signal as signal
import soundfile as sf

def batch_resample(input_folder, output_folder, target_sr):
    input_dir = Path(input_folder)
    output_dir = Path(output_folder)
    
    # Create the output directory if it doesn't exist
    output_dir.mkdir(exist_ok=True, parents=True)
    
    if not input_dir.exists():
        print(f"Error: The source folder '{input_dir}' does not exist.")
        return

    wav_files = list(input_dir.glob("*.mp3"))
    if not wav_files:
        print(f"No .wav files found inside the '{input_dir}' folder.")
        return

    print(f"Processing {len(wav_files)} files from '{input_dir}' -> '{output_dir}'")
    print(f"Target Sample Rate: {target_sr} Hz\n" + "-"*50)

    for file_path in wav_files:
        try:
            data, native_sr = sf.read(file_path)
            
            if native_sr != target_sr:
                # Polyphase resampling handles mono and stereo along axis 0
                resampled_data = signal.resample_poly(data, target_sr, native_sr)
                print(f"[Resampled] {file_path.name} ({native_sr} Hz -> {target_sr} Hz)")
            else:
                resampled_data = data
                print(f"[Copied]    {file_path.name} (Already {target_sr} Hz)")
            
            sf.write(output_dir / file_path.name, resampled_data, target_sr)
            
        except Exception as e:
            print(f"[Failed]    {file_path.name}. Error: {e}")

    print("-"*50 + f"\nBatch conversion complete! Output saved to: '{output_dir}'")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Batch resample WAV files in a directory using Scipy.")
    
    # Define external arguments
    parser.add_argument("-i", "--input", default="sf", help="Path to the original audio folder (default: sf)")
    parser.add_argument("-o", "--output", default="sf_resampled", help="Path to save resampled files (default: sf_resampled)")
    parser.add_argument("-r", "--rate", type=int, default=44100, help="Target sampling rate in Hz (default: 44100)")

    args = parser.parse_args()
    
    batch_resample(args.input, args.output, args.rate)
