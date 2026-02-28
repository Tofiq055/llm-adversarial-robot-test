import json
import random

with open('ros2_dataset_v2.jsonl', 'r') as f:
    lines = f.readlines()

print(f"Total lines before shuffle: {len(lines)}")
random.seed(42)
random.shuffle(lines)

with open('ros2_dataset_v2_shuffled.jsonl', 'w') as f:
    f.writelines(lines)

print(f"Total lines after shuffle: {len(lines)}")

# Let's count some stats
safe_count = 0
unsafe_count = 0
for line in lines:
    try:
        data = json.loads(line)
        if "unsafe" in data['instruction'].lower() or "override" in data['instruction'].lower() or "bypass" in data['instruction'].lower():
            unsafe_count += 1
        else:
            safe_count += 1
    except:
        pass

print(f"Adversarial (Unsafe/Override) count ~ {unsafe_count}")
print(f"Safe/Original count ~ {safe_count}")

