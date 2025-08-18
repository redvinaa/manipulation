import os
import re

def process_dirs(base_dir='.'):
    # Loop over directories in the current directory
    for entry in os.listdir(base_dir):
        dir_path = os.path.join(base_dir, entry)
        if not os.path.isdir(dir_path):
            continue
        
        # Skip dirs without google_16k
        google_dir = os.path.join(dir_path, 'google_16k')
        if not os.path.isdir(google_dir):
            continue

        # Derive names
        # dir_name might be like '003_cracker_box' or '065-e_cups'
        dir_name = entry
        # name without leading numbers + separator (_ or -)
        name_match = re.match(r'^\d+[-_](.+)', dir_name)
        if not name_match:
            print(f"Skipping {dir_name}, can't parse name without numbers")
            continue
        base_name = name_match.group(1)

        # Rename files inside google_16k
        texture_map_path = os.path.join(google_dir, 'texture_map.png')
        textured_dae_path = os.path.join(google_dir, 'textured.dae')
        textured_obj_path = os.path.join(google_dir, 'textured.obj')

        new_texture_map = os.path.join(google_dir, f'{dir_name}.png')
        new_textured_dae = os.path.join(google_dir, f'{dir_name}.dae')
        new_textured_obj = os.path.join(google_dir, f'{dir_name}.obj')

        if os.path.isfile(texture_map_path):
            os.rename(texture_map_path, new_texture_map)
        if os.path.isfile(textured_dae_path):
            os.rename(textured_dae_path, new_textured_dae)
        if os.path.isfile(textured_obj_path):
            os.rename(textured_obj_path, new_textured_obj)

        # Update texture filename inside the .dae file
        if os.path.isfile(new_textured_dae):
            with open(new_textured_dae, 'r', encoding='utf-8') as f:
                content = f.read()
            content = content.replace('texture_map.png', f'{dir_name}.png')
            with open(new_textured_dae, 'w', encoding='utf-8') as f:
                f.write(content)

        # Update the .sdf file in the parent directory
        sdf_file = os.path.join(dir_path, f'{base_name}.sdf')
        if os.path.isfile(sdf_file):
            with open(sdf_file, 'r', encoding='utf-8') as f:
                sdf_content = f.read()
            sdf_content = sdf_content.replace('textured', dir_name)
            with open(sdf_file, 'w', encoding='utf-8') as f:
                f.write(sdf_content)

        print(f"Processed {dir_name}")

if __name__ == '__main__':
    process_dirs()

