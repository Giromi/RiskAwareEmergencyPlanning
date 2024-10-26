import pickle
from selenium import webdriver
import undetected_chromedriver as uc
from fake_useragent import UserAgent
from lib.wait_button import quit_button
import os
import time

# 브라우저 설정
op = webdriver.ChromeOptions()
op.add_argument(f"user-agent={UserAgent.random}")
op.add_argument("user-data-dir=./")  # 사용자 데이터 유지
op.add_experimental_option("detach", True)
op.add_experimental_option("excludeSwitches", ["enable-logging"])

driver = uc.Chrome(chrome_options=op)

# OpenAI 로그인 페이지로 이동
driver.get('https://chat.openai.com/auth/login')

# 쿠키 파일 경로
cookie_file = "cookies.pkl"

# 쿠키 복원
if os.path.exists(cookie_file):
    with open(cookie_file, "rb") as f:
        cookies = pickle.load(f)
        for cookie in cookies:
            cookie.pop('domain', None)  # 쿠키에서 'domain' 필드를 제거
            try:
                driver.add_cookie(cookie)
            except Exception as e:
                print(f"쿠키 설정 중 오류 발생: {e}")
    print("쿠키 복원 완료. 페이지를 새로고침합니다.")
    driver.refresh()
else:
    print("쿠키 파일이 없습니다. 새로 로그인하십시오.")

# 사용자로 하여금 로그인 후 쿠키 저장을 위한 시간 제공
quit_button("q")
time.sleep(10)  # 사용자가 로그인할 수 있는 시간 제공

# 로그인 후 쿠키 저장
cookies = driver.get_cookies()
with open(cookie_file, "wb") as f:
    pickle.dump(cookies, f)
    print("로그인 후 쿠키 저장 완료.")

# 종료를 위한 quit_button 호출
