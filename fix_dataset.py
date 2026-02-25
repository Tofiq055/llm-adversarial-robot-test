import json
import os

def fix_dataset():
    input_file = 'ros2_dataset.jsonl'
    output_file = 'ros2_dataset_fixed.jsonl'
    
    print("Reading malformed dataset...")
    if not os.path.exists(input_file):
        print(f"File {input_file} not found!")
        return
        
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()
        
    print(f"Original content length: {len(content)} characters.")
    
    # Try multiple possible delimiters used in the broken file
    delimiters = ['}\\n{"instruction"', '}\\n{"instruction"', '}\\r\\n{"instruction"']
    blocks = [content]
    
    for delim in delimiters:
        if delim in content:
            blocks = content.split(delim)
            break
            
    if len(blocks) <= 1:
        # Fallback if the delimiter is something else
        blocks = content.split('}\\n{')
        if len(blocks) > 1:
            print("Found basic delimiter '}\\n{'")
    
    print(f"Found {len(blocks)} potential JSON blocks.")
    
    data = []
    for i, block in enumerate(blocks):
        # Restore the delimiter parts that were stripped during split
        if i > 0 and not block.startswith('"instruction'):
            block = '{"instruction"' + block
        elif i > 0:
            block = '{' + block
            
        if not block.endswith('}'):
            block = block + '}'
            
        try:
            item = json.loads(block)
            data.append(item)
        except json.JSONDecodeError as e:
            # Strip trailing/leading garbage and try one more time
            try:
                clean_block = block.strip()
                if clean_block.startswith('"{'): clean_block = clean_block[1:]
                if clean_block.endswith('}"'): clean_block = clean_block[:-1]
                item = json.loads(clean_block)
                data.append(item)
            except Exception as e2:
                print(f"Failed to parse block {i}: {e2}")

    print(f"Successfully loaded {len(data)} valid JSON items!")
    
    if len(data) > 0:
        print(f"Writing proper JSONLines to {output_file}...")
        with open(output_file, 'w', encoding='utf-8') as f_out:
            for item in data:
                f_out.write(json.dumps(item) + '\n')
        print("Data formatting complete! You can now resume fine-tuning.")
        # Copy original file to backup and replace with fixed version
        import shutil
        shutil.copy(input_file, input_file + '.bak')
        shutil.move(output_file, input_file)
        print(f"Backed up original to {input_file}.bak and overwrote {input_file} with fixed data.")
    else:
        print("Error: No valid JSON items could be recovered.")

if __name__ == '__main__':
    fix_dataset()
