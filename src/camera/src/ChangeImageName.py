import os

def rename_jpg_files(directory, new_name_base):
    # 지정된 경로의 파일 목록을 가져옵니다.
    files = os.listdir(directory)
    
    # JPG 파일만 필터링합니다.
    jpg_files = [f for f in files if f.lower().endswith('.jpg')]
    
    # 파일 이름을 변경합니다.
    for i, filename in enumerate(jpg_files):
        # 새로운 파일 이름 생성
        new_name = f"{new_name_base}_{i+1}.jpg"
        
        # 전체 경로 생성
        old_path = os.path.join(directory, filename)
        new_path = os.path.join(directory, new_name)
        
        # 파일 이름 변경
        os.rename(old_path, new_path)
        print(f"Renamed '{filename}' to '{new_name}'")

# 경로와 새로운 파일 이름의 기본을 설정합니다.
directory_path = '/home/erp42/Virtual_80'  # 실제 경로로 바꾸세요
new_name_base = 'Virtual_80'

# 함수 호출
rename_jpg_files(directory_path, new_name_base)