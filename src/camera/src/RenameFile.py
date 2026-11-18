import os

# 폴더 경로를 지정하세요
folder_path = '/home/kyd/Dataset_TrafficLight_240903/TrafficLight_4/Yellow/'

# 폴더 내 모든 파일 리스트를 가져옵니다
files = os.listdir(folder_path)

# JPG 파일만 필터링합니다
jpg_files = [f for f in files if f.lower().endswith('.jpg')]

# 파일 이름을 변경합니다
for index, filename in enumerate(jpg_files, start=1):
    # 새 파일 이름 생성
    new_name = f'TS4_5_{index}.jpg'
    
    # 기존 파일의 전체 경로
    old_file = os.path.join(folder_path, filename)
    
    # 새 파일의 전체 경로
    new_file = os.path.join(folder_path, new_name)
    
    # 파일 이름 변경
    os.rename(old_file, new_file)

print('파일 이름 변경 완료!')
