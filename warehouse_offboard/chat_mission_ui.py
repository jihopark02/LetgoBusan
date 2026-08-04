#!/usr/bin/env python3

import os
import threading
import time
import math

# 오토마타 직접 구현 방식 → OS IME가 키를 가로채지 않도록 비활성화
os.environ['XMODIFIERS'] = ''
os.environ['GTK_IM_MODULE'] = 'none'
os.environ['QT_IM_MODULE'] = 'none'

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from hangul_utils import join_jamos
    _HANGUL_OK = True
except ImportError:
    _HANGUL_OK = False

# ── 한글 오토마타 (OS IME 없이 직접 조합) ─────────────────────
class KoreanInput:
    CONS = {'r':'ㄱ','R':'ㄲ','s':'ㄴ','e':'ㄷ','E':'ㄸ','f':'ㄹ','a':'ㅁ','q':'ㅂ','Q':'ㅃ',
            't':'ㅅ','T':'ㅆ','d':'ㅇ','w':'ㅈ','W':'ㅉ','c':'ㅊ','z':'ㅋ','x':'ㅌ','v':'ㅍ','g':'ㅎ'}
    VOWELS = {'k':'ㅏ','o':'ㅐ','i':'ㅑ','O':'ㅒ','j':'ㅓ','p':'ㅔ','u':'ㅕ','P':'ㅖ','h':'ㅗ',
              'hk':'ㅘ','ho':'ㅙ','hl':'ㅚ','y':'ㅛ','n':'ㅜ','nj':'ㅝ','np':'ㅞ','nl':'ㅟ',
              'b':'ㅠ','m':'ㅡ','ml':'ㅢ','l':'ㅣ'}
    CONS_DOUBLE = {'rt':'ㄳ','sw':'ㄵ','sg':'ㄶ','fr':'ㄺ','fa':'ㄻ','fq':'ㄼ','ft':'ㄽ',
                   'fx':'ㄾ','fv':'ㄿ','fg':'ㅀ','qt':'ㅄ'}

    def __init__(self):
        self.han_mode = False
        self.han_buf = ''     # 조합 중인 로마자 버퍼

    def toggle(self):
        composed = self._compose(self.han_buf)
        self.han_buf = ''
        self.han_mode = not self.han_mode
        return composed

    def feed(self, ch):
        """키 입력 → (확정된 텍스트, 현재 조합 미리보기)"""
        if not self.han_mode or not _HANGUL_OK:
            return ch, ''
        self.han_buf += ch
        preview = self._compose(self.han_buf)
        return '', preview

    def commit(self):
        """조합 중인 버퍼 확정"""
        result = self._compose(self.han_buf)
        self.han_buf = ''
        return result

    def backspace(self):
        """한 글자 지우기 → (True=한글버퍼에서 처리, False=일반 백스페이스)"""
        if self.han_mode and self.han_buf:
            self.han_buf = self.han_buf[:-1]
            preview = self._compose(self.han_buf) if self.han_buf else ''
            return True, preview
        return False, ''

    @classmethod
    def _compose(cls, text):
        if not text or not _HANGUL_OK:
            return text
        result = ''
        vc = ''
        for t in text:
            if t in cls.CONS:   vc += 'c'
            elif t in cls.VOWELS: vc += 'v'
            else: vc += '!'
        vc = vc.replace('cvv','fVV').replace('cv','fv').replace('cc','dd')
        i = 0
        while i < len(text):
            v = vc[i]; t = text[i]; j = 1
            try:
                if v in ('f','c'):   result += cls.CONS[t]
                elif v == 'V':       result += cls.VOWELS[text[i:i+2]]; j=2
                elif v == 'v':       result += cls.VOWELS[t]
                elif v == 'd':       result += cls.CONS_DOUBLE[text[i:i+2]]; j=2
                else:                result += t
            except Exception:
                result += t
            i += j
        return join_jamos(result)


# ── 상태 → 한글 매핑 (이모지 없음) ───────────────────────────
STATUS_MAP = {
    'WAITING_FOR_COMMAND':              '[대기] 명령 대기 중',
    'MISSION_REJECTED:BUSY':            '[거부] 미션 진행 중',
    'MISSION_REJECTED:UNKNOWN_TARGET':  '[거부] 알 수 없는 목표',
    'MISSION_REJECTED:POSITION_INVALID':'[거부] 위치 오류',
    'PRELAND_SETTLE':                   '[착륙] 착륙 준비 중',
    'ARUCO_LAND_TIMEOUT':               '[경고] 착륙 타임아웃',
}

def friendly_status(raw: str) -> str:
    if raw in STATUS_MAP:
        return STATUS_MAP[raw]
    if raw.startswith('MISSION_STARTED:'):
        return f"[시작] 미션: {raw.split(':', 1)[1]}"
    if raw.startswith('MISSION_FINISHED:'):
        return f"[완료] 미션: {raw.split(':', 1)[1]}"
    if raw.startswith('SCAN_START:'):
        return f"[스캔] 시작: {raw.split(':', 1)[1]}"
    if raw.startswith('SCAN_LAYER:'):
        return f"[스캔] 층: {raw.split(':', 1)[1]}"
    if raw.startswith('SCAN_NEXT:'):
        return f"[스캔] 다음: {raw.split(':', 1)[1]}"
    if raw.startswith('SCAN_DONE:'):
        return f"[완료] 스캔: {raw.split(':', 1)[1]}"
    if raw.startswith('SCAN_TIMEOUT:'):
        return f"[타임] 스캔: {raw.split(':', 1)[1]}"
    if raw.startswith('INVENTORY_ACCEPTED:'):
        return f"[재고] 확인: {raw.split(':', 1)[1]}"
    if raw.startswith('명령 수신:'):
        return f"[수신] {raw.split(':', 1)[1].strip()}"
    return raw


def status_color(raw: str, C: dict):
    if any(k in raw for k in ['FINISHED', 'WAITING', 'ACCEPTED', 'DONE', '완료', '대기']):
        return C['ok']
    if any(k in raw for k in ['REJECTED', 'TIMEOUT', 'ERROR', '거부', '경고', '타임']):
        return C['err']
    if any(k in raw for k in ['STARTED', 'SCAN', 'MOVE', 'TAKEOFF', 'RETURN', 'PRELAND', '시작', '스캔', '착륙', '수신']):
        return C['run']
    return C['gray']


class MissionUiBridge(Node):
    def __init__(self):
        super().__init__('mission_ui_bridge')

        # llm_node로 자연어 입력 전달
        self.user_input_pub = self.create_publisher(String, '/llm/user_input', 10)
        # llm_node의 응답 텍스트 구독
        self.response_sub = self.create_subscription(
            String, '/llm/response_text', self.response_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/mission_status_text', self.status_callback, 10)

        self.latest_status = 'WAITING_FOR_COMMAND'
        # (friendly_text, raw, timestamp) 리스트
        self.status_history = []
        # llm_node로부터 받은 응답 대기열
        self.pending_responses = []
        self.get_logger().info('Mission UI Bridge initialized')

    def status_callback(self, msg: String):
        self.latest_status = msg.data
        friendly = friendly_status(msg.data)
        ts = time.strftime('%H:%M:%S')
        # 연속 중복 무시
        if not self.status_history or self.status_history[0][0] != friendly:
            self.status_history.insert(0, (friendly, msg.data, ts))
        self.status_history = self.status_history[:40]

    def response_callback(self, msg: String):
        """llm_node의 자연어 응답 수신"""
        self.pending_responses.append(msg.data)

    def publish_user_input(self, text: str):
        msg = String()
        msg.data = text
        self.user_input_pub.publish(msg)
        self.get_logger().info(f'Published user input to llm_node: {text}')


class LLMInterface:
    USER = 'user'
    BOT  = 'bot'
    SYS  = 'sys'

    def __init__(self, ros_node: MissionUiBridge):
        self.ros_node = ros_node

        pygame.init()

        self.W, self.H = 960, 700
        self.screen = pygame.display.set_mode((self.W, self.H), pygame.RESIZABLE)
        pygame.display.set_caption("Drone Mission Chat UI")
        pygame.key.start_text_input()  # 윈도우 생성 후 IME 바인딩

        # 폰트 (이모지 없는 시스템 폰트)
        self.font_title  = pygame.font.SysFont('Noto Sans CJK KR', 22, bold=True)
        self.font_chat   = pygame.font.SysFont('Noto Sans CJK KR', 20)
        self.font_small  = pygame.font.SysFont('Noto Sans CJK KR', 17)
        self.font_tag    = pygame.font.SysFont('Noto Sans CJK KR', 16, bold=True)
        self.font_status = pygame.font.SysFont('Noto Sans CJK KR', 18, bold=True)
        self.font_input  = pygame.font.SysFont('Noto Sans CJK KR', 20)

        self.C = {
            'bg':           (52,  53,  65),
            'sidebar':      (28,  29,  32),
            'header':       (60,  61,  75),
            'user_bubble':  (16, 150, 115),
            'bot_bubble':   (62,  64,  78),
            'sys_bubble':   (42,  43,  55),
            'input_bg':     (60,  61,  75),
            'input_border': (90,  92, 110),
            'divider':      (65,  67,  82),
            'send':         (16, 150, 115),
            'send_hover':   (20, 175, 138),
            'white':        (230, 232, 235),
            'gray':         (150, 152, 168),
            'ok':           ( 72, 199, 142),
            'err':          (235,  80,  80),
            'run':          ( 80, 148, 255),
            'warn':         (255, 185,   0),
            # 태그 배경
            'tag_ok':       ( 30,  90,  60),
            'tag_err':      ( 90,  30,  30),
            'tag_run':      ( 30,  55, 110),
            'tag_gray':     ( 50,  50,  65),
        }

        # 채팅 로그: (type, text, ts)
        self.chat_log = []
        self._add_sys('드론 미션 시스템이 준비되었습니다.')
        self._add_sys('사용 가능한 선반: A-01  A-02  A-03  A-04')

        self.input_text   = ''
        self.han          = KoreanInput()
        self.han_preview  = ''   # 조합 중 미리보기
        self.cursor_vis   = True
        self.cursor_tick  = 0
        self.chat_scroll  = 0
        self.log_scroll   = 0
        self.max_chat_scroll = 0
        self.max_log_scroll  = 0

        self.send_btn_rect = pygame.Rect(0, 0, 1, 1)
        self.spinner_angle = 0
        self._prev_status  = ''

    # ── 채팅 로그 헬퍼 ────────────────────────────────────────
    def _add_sys(self, t):
        self.chat_log.append((self.SYS, t, time.strftime('%H:%M')))

    def _add_bot(self, t):
        self.chat_log.append((self.BOT, t, time.strftime('%H:%M')))

    def _add_user(self, t):
        self.chat_log.append((self.USER, t, time.strftime('%H:%M')))

    # ── 텍스트 줄바꿈 ─────────────────────────────────────────
    def _wrap(self, text, font, max_w):
        words = text.split(' ')
        lines, cur = [], ''
        for w in words:
            test = (cur + ' ' + w).strip()
            if font.size(test)[0] <= max_w:
                cur = test
            else:
                if cur:
                    lines.append(cur)
                cur = w
        if cur:
            lines.append(cur)
        return lines or ['']

    # ── 태그 그리기 ([완료] 같은 뱃지) ───────────────────────
    def _tag_color(self, text):
        if any(k in text for k in ['완료', '대기', '재고', 'ok']):
            return self.C['tag_ok'], self.C['ok']
        if any(k in text for k in ['거부', '경고', '타임', 'err']):
            return self.C['tag_err'], self.C['err']
        if any(k in text for k in ['시작', '스캔', '착륙', '수신', '다음', 'run']):
            return self.C['tag_run'], self.C['run']
        return self.C['tag_gray'], self.C['gray']

    def _draw_tag_line(self, surface, text, x, y, max_w):
        """[태그] 텍스트 형식으로 태그 배경 강조해서 그리기"""
        if text.startswith('[') and ']' in text:
            bracket_end = text.index(']') + 1
            tag = text[:bracket_end]
            rest = text[bracket_end:].strip()
        else:
            tag = ''
            rest = text

        bg, fg = self._tag_color(text)
        tx = x

        if tag:
            tag_surf = self.font_tag.render(tag, True, fg)
            tag_rect = pygame.Rect(tx - 2, y - 1, tag_surf.get_width() + 6, tag_surf.get_height() + 2)
            pygame.draw.rect(surface, bg, tag_rect, border_radius=3)
            surface.blit(tag_surf, (tx, y))
            tx += tag_surf.get_width() + 8

        if rest:
            rest_surf = self.font_small.render(rest[:40], True, self.C['white'])
            surface.blit(rest_surf, (tx, y))

        return self.font_small.get_linesize() + 3

    # ── 말풍선 ────────────────────────────────────────────────
    def _draw_bubble(self, surface, btype, text, ts, y, chat_w):
        pad = 12
        max_bw = int(chat_w * 0.70)
        font = self.font_chat
        ts_font = self.font_small

        lines = self._wrap(text, font, max_bw - pad * 2)
        lh = font.get_linesize()
        bh = lh * len(lines) + pad * 2
        bw = max(min(max(font.size(l)[0] for l in lines) + pad * 2, max_bw), 60)

        if btype == self.USER:
            bx = chat_w - bw - 12
            bc = self.C['user_bubble']
            tc = self.C['white']
            r  = 14
        elif btype == self.BOT:
            bx = 12
            bc = self.C['bot_bubble']
            tc = self.C['white']
            r  = 14
        else:
            bx = 12
            bc = self.C['sys_bubble']
            tc = self.C['gray']
            r  = 6

        pygame.draw.rect(surface, bc, pygame.Rect(bx, y, bw, bh), border_radius=r)
        for i, line in enumerate(lines):
            surface.blit(font.render(line, True, tc), (bx + pad, y + pad + i * lh))

        ts_surf = ts_font.render(ts, True, self.C['gray'])
        if btype == self.USER:
            surface.blit(ts_surf, (bx + bw - ts_surf.get_width() - 2, y + bh + 2))
        else:
            surface.blit(ts_surf, (bx + 2, y + bh + 2))

        return bh + ts_font.get_linesize() + 8

    # ── 메인 렌더 ─────────────────────────────────────────────
    def render_ui(self):
        W, H = self.screen.get_size()
        sidebar_w = 240
        chat_x    = sidebar_w
        chat_w    = W - sidebar_w
        header_h  = 46
        input_h   = 58
        chat_area_h = H - header_h - input_h

        self.screen.fill(self.C['bg'])

        # ════════════════════════════════
        # 사이드바
        # ════════════════════════════════
        pygame.draw.rect(self.screen, self.C['sidebar'], (0, 0, sidebar_w, H))
        pygame.draw.line(self.screen, self.C['divider'], (sidebar_w, 0), (sidebar_w, H), 1)

        # 타이틀
        t = self.font_title.render('Mission Control', True, self.C['white'])
        self.screen.blit(t, (12, 13))
        pygame.draw.line(self.screen, self.C['divider'], (8, 40), (sidebar_w - 8, 40), 1)

        # 현재 상태 + 스피너
        raw = self.ros_node.latest_status
        friendly = friendly_status(raw)
        sc = status_color(raw, self.C)
        is_running = any(k in raw for k in
                         ['STARTED', 'SCAN', 'MOVE', 'TAKEOFF', 'RETURN', 'PRELAND'])

        sy = 50
        # 스피너 or 원형 인디케이터
        if is_running:
            self.spinner_angle = (self.spinner_angle + 5) % 360
            for i in range(8):
                a = math.radians(self.spinner_angle + i * 45)
                alpha_c = tuple(int(c * (i + 1) / 8) for c in sc)
                ex = int(14 + 7 * math.cos(a))
                ey = int(sy + 8 + 7 * math.sin(a))
                pygame.draw.circle(self.screen, alpha_c, (ex, ey), 2)
        else:
            pygame.draw.circle(self.screen, sc, (14, sy + 8), 5)

        # 상태 텍스트
        label = self.font_status.render(
            '진행 중' if is_running else ('대기' if 'WAITING' in raw else '완료'),
            True, sc)
        self.screen.blit(label, (26, sy))
        sy += label.get_height() + 4

        # 현재 상태 상세
        for line in self._wrap(friendly, self.font_small, sidebar_w - 18):
            self.screen.blit(self.font_small.render(line, True, sc), (10, sy))
            sy += self.font_small.get_linesize() + 1

        # 구분선
        sy += 6
        pygame.draw.line(self.screen, self.C['divider'], (8, sy), (sidebar_w - 8, sy), 1)
        sy += 8

        # 시스템 로그 타이틀
        self.screen.blit(
            self.font_status.render('시스템 로그', True, self.C['gray']), (10, sy))
        sy += self.font_status.get_linesize() + 4

        log_area_h = H - sy - 4
        log_surface = pygame.Surface((sidebar_w, max(log_area_h, 2000)))
        log_surface.fill(self.C['sidebar'])

        ly = 4
        for friendly_t, raw_t, ts_t in self.ros_node.status_history:
            # 타임스탬프
            ts_surf = self.font_small.render(ts_t, True, self.C['gray'])
            log_surface.blit(ts_surf, (4, ly))
            ly += ts_surf.get_height() + 1
            # 태그 + 텍스트
            ly += self._draw_tag_line(log_surface, friendly_t, 4, ly, sidebar_w - 8)
            # 구분선
            pygame.draw.line(log_surface, self.C['divider'], (4, ly + 1), (sidebar_w - 8, ly + 1), 1)
            ly += 5

        self.max_log_scroll = max(0, ly - log_area_h)
        self.log_scroll = min(self.log_scroll, self.max_log_scroll)
        self.screen.blit(log_surface, (0, sy),
                         area=pygame.Rect(0, self.log_scroll, sidebar_w, log_area_h))

        # ════════════════════════════════
        # 헤더
        # ════════════════════════════════
        pygame.draw.rect(self.screen, self.C['header'], (chat_x, 0, chat_w, header_h))
        pygame.draw.line(self.screen, self.C['divider'], (chat_x, header_h), (W, header_h), 1)
        ht = self.font_title.render('Drone Natural Language Control', True, self.C['white'])
        self.screen.blit(ht, (chat_x + 14, (header_h - ht.get_height()) // 2))

        # ════════════════════════════════
        # 채팅 영역
        # ════════════════════════════════
        chat_surface = pygame.Surface((chat_w, max(chat_area_h, 3000)))
        chat_surface.fill(self.C['bg'])

        cy = 12
        for btype, text, ts in self.chat_log:
            cy += self._draw_bubble(chat_surface, btype, text, ts, cy, chat_w)
            cy += 6

        self.max_chat_scroll = max(0, cy - chat_area_h + 20)
        self.chat_scroll = min(self.chat_scroll, self.max_chat_scroll)
        self.screen.blit(chat_surface, (chat_x, header_h),
                         area=pygame.Rect(0, self.chat_scroll, chat_w, chat_area_h))

        # ════════════════════════════════
        # 입력창
        # ════════════════════════════════
        iy = H - input_h
        pygame.draw.rect(self.screen, self.C['header'], (chat_x, iy, chat_w, input_h))
        pygame.draw.line(self.screen, self.C['divider'], (chat_x, iy), (W, iy), 1)

        btn_w, btn_h = 68, 36
        btn_x = W - btn_w - 10
        btn_y = iy + (input_h - btn_h) // 2
        self.send_btn_rect = pygame.Rect(btn_x, btn_y, btn_w, btn_h)

        mx, my = pygame.mouse.get_pos()
        hover = self.send_btn_rect.collidepoint(mx, my)
        pygame.draw.rect(self.screen,
                         self.C['send_hover'] if hover else self.C['send'],
                         self.send_btn_rect, border_radius=8)
        bs = self.font_status.render('전송 >', True, self.C['white'])
        self.screen.blit(bs, (btn_x + (btn_w - bs.get_width()) // 2,
                               btn_y + (btn_h - bs.get_height()) // 2))

        inp_rect = pygame.Rect(chat_x + 10, btn_y, btn_x - chat_x - 16, btn_h)
        pygame.draw.rect(self.screen, self.C['input_bg'], inp_rect, border_radius=8)
        pygame.draw.rect(self.screen, self.C['input_border'], inp_rect, 1, border_radius=8)

        # 커서 깜빡임
        self.cursor_tick += 1
        if self.cursor_tick % 28 == 0:
            self.cursor_vis = not self.cursor_vis
        display = self.input_text + self.han_preview + ('|' if self.cursor_vis else '')

        it = self.font_input.render(display, True, self.C['white'])
        clip_w = inp_rect.width - 16
        if it.get_width() > clip_w:
            it = it.subsurface((it.get_width() - clip_w, 0, clip_w, it.get_height()))
        self.screen.blit(it, (inp_rect.x + 10, inp_rect.y + (btn_h - it.get_height()) // 2))

        if not self.input_text and not self.han_preview:
            ph = self.font_input.render('목적지를 입력하세요 (예: A-01, a-01 가줘)', True, self.C['gray'])
            self.screen.blit(ph, (inp_rect.x + 10, inp_rect.y + (btn_h - ph.get_height()) // 2))

    # ── ROS 상태 → 채팅창 동기화 ──────────────────────────────
    def _sync_status(self):
        raw = self.ros_node.latest_status
        if raw == self._prev_status:
            pass
        else:
            self._prev_status = raw
            if any(k in raw for k in ['MISSION_FINISHED', 'MISSION_STARTED',
                                       'MISSION_REJECTED', 'WAITING_FOR_COMMAND']):
                self._add_sys(friendly_status(raw))
                self.chat_scroll = self.max_chat_scroll

        # llm_node 응답 텍스트를 채팅창에 반영
        while self.ros_node.pending_responses:
            resp = self.ros_node.pending_responses.pop(0)
            # 마지막 "처리 중" 메시지를 실제 응답으로 교체
            for i in range(len(self.chat_log) - 1, -1, -1):
                if self.chat_log[i][0] == self.BOT and 'GPT가 처리 중' in self.chat_log[i][1]:
                    self.chat_log[i] = (self.BOT, resp, self.chat_log[i][2])
                    break
            else:
                self._add_bot(resp)
            self.chat_scroll = self.max_chat_scroll

    def _send(self):
        command = self.input_text.strip()
        if not command:
            return
        self.input_text = ''
        self._add_user(command)
        self.chat_scroll = self.max_chat_scroll

        # llm_node로 자연어 전달 (응답은 response_callback으로 비동기 수신)
        self.ros_node.publish_user_input(command)
        self._add_bot('명령을 전송했습니다. GPT가 처리 중입니다...')
        self.chat_scroll = self.max_chat_scroll

    def run(self):
        clock = pygame.time.Clock()
        running = True
        time.sleep(0.5)
        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.VIDEORESIZE:
                        self.screen = pygame.display.set_mode(event.size, pygame.RESIZABLE)
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False
                        elif event.key == pygame.K_RETURN:
                            self.input_text += self.han.commit()
                            self.han_preview = ''
                            self._send()
                        elif event.key == pygame.K_BACKSPACE:
                            handled, self.han_preview = self.han.backspace()
                            if not handled:
                                self.input_text = self.input_text[:-1]
                        elif (event.mod & pygame.KMOD_LSHIFT) and event.key == pygame.K_SPACE:
                            # Left Shift + Space: 한/영 전환
                            self.input_text += self.han.toggle()
                            self.han_preview = ''
                        elif self.han.han_mode and event.unicode and event.unicode.isprintable():
                            committed, self.han_preview = self.han.feed(event.unicode)
                            self.input_text += committed
                    elif event.type == pygame.TEXTINPUT:
                        if not self.han.han_mode:
                            self.input_text += event.text
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1 and self.send_btn_rect.collidepoint(event.pos):
                            self._send()
                        elif event.button == 4:
                            # 마우스가 사이드바 위면 로그 스크롤, 아니면 채팅 스크롤
                            if pygame.mouse.get_pos()[0] < 240:
                                self.log_scroll = max(0, self.log_scroll - 30)
                            else:
                                self.chat_scroll = max(0, self.chat_scroll - 30)
                        elif event.button == 5:
                            if pygame.mouse.get_pos()[0] < 240:
                                self.log_scroll = min(self.max_log_scroll, self.log_scroll + 30)
                            else:
                                self.chat_scroll = min(self.max_chat_scroll, self.chat_scroll + 30)

                self._sync_status()
                self.render_ui()
                pygame.display.flip()
                clock.tick(30)
        finally:
            pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MissionUiBridge()
        thread = threading.Thread(target=lambda: rclpy.spin(node))
        thread.daemon = True
        thread.start()
        gui = LLMInterface(node)
        gui.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
