import selenium
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
from time import sleep
import undetected_chromedriver as uc
from selenium.webdriver.support.ui import WebDriverWait
from fake_useragent import UserAgent
from selenium.webdriver.support import expected_conditions as EC
from selenium.common.exceptions import TimeoutException
from selenium.webdriver.common.action_chains import ActionChains
from lib.wait_button import quit_button

import pickle
# 페이지 새로고침 후 로그인된 상태로 유지
import os
from dotenv import load_dotenv

# 모니터 해상도를 50% 크기로 설정
screen_width = 1280
screen_height = 800

# 원하는 브라우저 크기 (50% 축소)
browser_width = int(screen_width * 0.5)
browser_height = int(screen_height)

load_dotenv()
MAIL = os.getenv("MAIL")
PASSWORD = os.getenv("PASSWORD")
# bring the txt
with open('prompt.txt', 'r') as f:
    # read newlines
    prompt = f.read()
    f.close()
print(prompt)  # 개행이나 기타 특수 문자가 시각적으로 확인됨

op = webdriver.ChromeOptions()
op.add_argument(f"user-agent={UserAgent.random}")
op.add_argument("user-data-dir=./")
op.add_experimental_option("detach", True)
op.add_experimental_option("excludeSwitches", ["enable-logging"])

driver = uc.Chrome(chrome_options=op)
driver.set_window_size(browser_width, browser_height)
# driver.execute_script("document.body.style.zoom = '0.5'")  # 50% 축소
# PATH = "path you your chrome driver"
print("브라우저 크기 설정 완료")


driver.get('https://chat.openai.com/auth/login')

# 쿠키 파일 경로
cookie_file = "cookies.pkl"

login = False
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
    login = True
else:
    print("쿠키 파일이 없습니다. 새로 로그인하십시오.")

wait = WebDriverWait(driver, 10)

# 대기 시간 설정
if not login:
    sleep(3)
    try:
        # data-testid 속성을 사용하여 로그인 버튼을 찾음

        # PC
        # login_button = wait.until(EC.element_to_be_clickable((By.CSS_SELECTOR, "button[data-testid='login-button']"))) 
        # Mobile
        login_button = wait.until(
            EC.element_to_be_clickable((By.CSS_SELECTOR, "button[data-testid='mobile-login-button']"))
        )
        sleep(1)
        login_button.click()
        print("로그인 버튼 클릭 성공!")
    except TimeoutException:
        print("로그인 버튼을 찾을 수 없습니다.")


    try:
        # social-text 클래스가 있는 span에서 'Continue with Google' 텍스트가 포함된 요소를 CSS Selector로 선택
        google_login_button = wait.until(EC.element_to_be_clickable((By.CSS_SELECTOR, "button.social-btn span.social-text")))
        sleep(1)
        # 텍스트 확인 후 클릭
        if google_login_button.text == "Continue with Google":
            google_login_button.click()
            print("Google 로그인 버튼 클릭 성공!")
    except TimeoutException:
        print("Google 로그인 버튼을 찾을 수 없습니다.")

    try:
        # 'id' 속성으로 이메일 입력란을 찾음
        email_input = wait.until(EC.presence_of_element_located((By.ID, "identifierId")))
        sleep(1)
        
        # 이메일 입력
        email_input.send_keys(MAIL)
        email_input.send_keys(Keys.ENTER)
        print("이메일 입력 후 엔터!")
        
    except TimeoutException:
        print("이메일 입력란을 찾을 수 없습니다.")
# q 키 입력을 기다리는 루프

    try:
        # 'name' 속성으로 비밀번호 입력란을 찾음
        password_input = wait.until(EC.presence_of_element_located((By.NAME, "Passwd")))
        sleep(1)
        
        # 비밀번호 입력
        password_input.send_keys(PASSWORD)
        sleep(1)
        password_input.send_keys(Keys.ENTER)
        print("비밀번호 입력 후 엔터!")

    except TimeoutException:
        print("비밀번호 입력란을 찾을 수 없습니다.")

sleep(3)

try:
    # 1. 사이드바 열기 버튼을 클릭
    open_sidebar_button = wait.until(EC.element_to_be_clickable((By.CSS_SELECTOR, "button[data-testid='open-sidebar-button']")))
    open_sidebar_button.click()
    print("사이드바 열기 버튼 클릭 성공!")

except TimeoutException:
    print("요소를 찾을 수 없습니다. 프로세스를 다시 확인하세요.")

try:
    sleep(3)
    # 모든 채팅방 목록을 가져옴
    chat_list = driver.find_elements(By.XPATH, "//div[@class='relative grow overflow-hidden whitespace-nowrap']")

    # 'Moral Machine' 텍스트가 포함된 채팅방을 찾아 클릭
    for chat in chat_list:
        if 'Moral Machine' in chat.text:
            actions = ActionChains(driver)
            actions.move_to_element(chat).perform()  # 해당 채팅방으로 스크롤
            sleep(1)  # 스크롤 후 잠시 대기
            chat.click()  # 채팅방 클릭
            print("Moral Machine 채팅방 클릭 성공!")
            break
    else:
        print("Moral Machine 채팅방을 찾을 수 없습니다.")
    
except TimeoutException:
    print("채팅방을 찾는 데 시간이 초과되었습니다.")
except selenium.common.exceptions.ElementNotInteractableException:
    print("요소가 클릭할 수 없는 상태입니다. 스크롤을 시도한 후 클릭하세요.")

try:

    sleep(3)

    # 채팅 입력란이 나타날 때까지 기다린 후, 메시지 입력
    chat_input = wait.until(EC.presence_of_element_located((By.XPATH, "//p[@data-placeholder='Message ChatGPT']")))
    formatted_prompt = prompt.replace("\n", "<br>")  # 개행을 <br> 태그로 변환
    chat_input.send_keys(formatted_prompt)
    print("메시지 전송 성공!")

    # 5. 메시지 전송 버튼을 찾고 클릭
    send_button = wait.until(EC.element_to_be_clickable((By.CSS_SELECTOR, "button[data-testid='send-button']")))
    if send_button.get_attribute("data-state") != "closed":
        send_button.click()
        print("메시지 전송 버튼 클릭 성공!")
    else:
        print("메시지 전송 버튼이 비활성화 상태입니다.")

except TimeoutException:
    print("요소를 찾을 수 없습니다. 프로세스를 다시 확인하세요.")

try:
    # Stop streaming 버튼을 기다림
    stop_button = wait.until(
        EC.presence_of_element_located((By.CSS_SELECTOR, "button[data-testid='stop-button']"))
    )
    print("Stop 버튼이 발견되었습니다.")

    while True:
        try:
            # 'Send' 버튼이 나타날 때까지 대기
            send_button = wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "button[data-testid='send-button']")))
            if send_button:
                print("Send 버튼이 발견되었습니다. 메시지를 가져옵니다...")
                break  # 버튼이 나타나면 루프를 탈출
        except TimeoutException:
            print("Send 버튼을 찾지 못했습니다. 다시 시도합니다...")
            sleep(1)  # 일정 시간 기다렸다가 다시 시도
except TimeoutException:
    print("요소를 찾을 수 없습니다.")

try:
    # 모든 'assistant'가 작성한 메시지 요소를 가져옴
    messages = driver.find_elements(By.XPATH, "//div[@data-message-author-role='assistant']")
    # 메시지가 존재할 경우, 가장 마지막 메시지를 가져옴
    if messages:
        last_message = messages[-1].text
        print("\n\n\n\n\n@@@@@@ 가져온 마지막 메시지 @@@@@@", last_message, '@@@@@@@@@@ END @@@@@@@@@@', sep="\n")
    else:
        print("assistant의 메시지를 찾을 수 없습니다.")

except TimeoutException:
    print("메시지를 찾는 데 시간이 초과되었습니다.")
quit_button('q')
print("프로그램 종료")
