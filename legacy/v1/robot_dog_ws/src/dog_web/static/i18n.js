/* RobotDogQwen - i18n (EN/RU) */
const I18N = {
  ru: {
    // Nav
    'nav.dashboard': 'Панель',
    'nav.control': 'Управление',
    'nav.state': 'Состояние',
    'nav.calibration': 'Калибровка',

    // Dashboard
    'dash.title': 'Панель управления',
    'dash.joints': 'Позиции суставов',
    'dash.quick': 'Быстрые действия',
    'dash.servo_on': 'Включить сервы',
    'dash.servo_off': 'Выключить сервы',
    'dash.estop': 'АВАРИЙНАЯ ОСТАНОВКА',
    'dash.estop_release': '🔓 Снять блокировку',
    'dash.movement': 'Движение',
    'dash.log': 'Журнал событий',
    'dash.loaded': 'Панель загружена',

    // Control
    'ctrl.title': 'Управление',
    'ctrl.joystick': 'Джойстик',
    'ctrl.joystick_hint': 'Тяните: X = вбок, Y = вперёд/назад',
    'ctrl.dpad': 'D-Pad',
    'ctrl.fwd': '↑ Вперёд',
    'ctrl.back': '↓ Назад',
    'ctrl.left': '← Влево',
    'ctrl.right': '→ Вправо',
    'ctrl.stop': '■ СТОП',
    'ctrl.rotation': 'Поворот',
    'ctrl.ccw': '↺ Против ч.с.',
    'ctrl.cw': '↻ По ч.с.',
    'ctrl.speed': 'Скорость',
    'ctrl.servo_on': '✅ Включить сервы',
    'ctrl.servo_off': '❌ Выключить сервы',
    'ctrl.estop': '🚨 АВАРИЙНАЯ ОСТАНОВКА',
    'ctrl.estop_release': '🔓 Снять блокировку',
    'ctrl.feedback': 'Обратная связь',
    'ctrl.loaded': 'Страница управления загружена',

    // State
    'state.title': 'Состояние',
    'state.lf': 'Передняя левая (LF)',
    'state.rf': 'Передняя правая (RF)',
    'state.lr': 'Задняя левая (LR)',
    'state.rr': 'Задняя правая (RR)',
    'state.raw': 'Данные суставов',
    'state.num': '№',
    'state.joint': 'Сустав',
    'state.pos_rad': 'Позиция (рад)',
    'state.pos_deg': 'Позиция (град)',
    'state.visual': 'Визуально',
    'state.update': 'Обновление: -- Гц',
    'state.hip': 'Бедро',
    'state.thigh': 'Бедро (привод)',
    'state.calf': 'Голень',

    // Calibration
    'cal.title': 'Калибровка',
    'cal.profiles': 'Профили калибровки',
    'cal.save': '💾 Сохранить текущий',
    'cal.apply': '⚡ Применить к роботу',
    'cal.load_default': '📂 Загрузить по умолчанию',
    'cal.reset': '🔄 Сбросить всё',
    'cal.profile_name': 'Имя профиля...',
    'cal.save_as': 'Сохранить как',
    'cal.lf': 'Передняя левая',
    'cal.rf': 'Передняя правая',
    'cal.lr': 'Задняя левая',
    'cal.rr': 'Задняя правая',
    'cal.hip': 'Бедро (Roll)',
    'cal.thigh': 'Бедро (Pitch)',
    'cal.calf': 'Голень (Колено)',
    'cal.loaded': 'Страница калибровки загружена',
    'cal.saved': 'Профиль сохранён: ',
    'cal.loaded_profile': 'Профиль загружен: ',
    'cal.reset_confirm': 'Сбросить всю калибровку?',
    'cal.reset_done': 'Калибровка сброшена',
    'cal.applied': 'Калибровка применена к роботу',

    // Common
    'common.ws': 'WebSocket',
    'common.estop': 'E-Stop',
    'common.ros': 'ROS2',
    'common.servos': 'Сервы',
  },
  en: {
    // Nav
    'nav.dashboard': 'Dashboard',
    'nav.control': 'Control',
    'nav.state': 'State',
    'nav.calibration': 'Calibration',

    // Dashboard
    'dash.title': 'Dashboard',
    'dash.joints': 'Joint Positions',
    'dash.quick': 'Quick Actions',
    'dash.servo_on': 'Enable Servos',
    'dash.servo_off': 'Disable Servos',
    'dash.estop': 'EMERGENCY STOP',
    'dash.movement': 'Movement',
    'dash.log': 'Event Log',
    'dash.loaded': 'Dashboard loaded',

    // Control
    'ctrl.title': 'Control',
    'ctrl.joystick': 'Joystick Control',
    'ctrl.joystick_hint': 'Drag to move: X = lateral, Y = forward/back',
    'ctrl.dpad': 'D-Pad',
    'ctrl.fwd': '↑ Fwd',
    'ctrl.back': '↓ Back',
    'ctrl.left': '← Left',
    'ctrl.right': '→ Right',
    'ctrl.stop': '■ STOP',
    'ctrl.rotation': 'Rotation',
    'ctrl.ccw': '↺ CCW',
    'ctrl.cw': '↻ CW',
    'ctrl.speed': 'Speed',
    'ctrl.servo_on': '✅ Enable Servos',
    'ctrl.servo_off': '❌ Disable Servos',
    'ctrl.estop': '🚨 EMERGENCY STOP',
    'ctrl.estop_release': '🔓 Release E-Stop',
    'ctrl.feedback': 'Live Command Feedback',
    'ctrl.loaded': 'Control page loaded',

    // State
    'state.title': 'State',
    'state.lf': 'Left Front (LF)',
    'state.rf': 'Right Front (RF)',
    'state.lr': 'Left Rear (LR)',
    'state.rr': 'Right Rear (RR)',
    'state.raw': 'Raw Joint Data',
    'state.num': '#',
    'state.joint': 'Joint',
    'state.pos_rad': 'Position (rad)',
    'state.pos_deg': 'Position (deg)',
    'state.visual': 'Visual',
    'state.update': 'Update: -- Hz',
    'state.hip': 'Hip',
    'state.thigh': 'Thigh',
    'state.calf': 'Calf',

    // Calibration
    'cal.title': 'Calibration',
    'cal.profiles': 'Calibration Profiles',
    'cal.save': '💾 Save Current',
    'cal.apply': '⚡ Apply to Robot',
    'cal.load_default': '📂 Load Default',
    'cal.reset': '🔄 Reset All',
    'cal.profile_name': 'Profile name...',
    'cal.save_as': 'Save As',
    'cal.lf': 'Left Front',
    'cal.rf': 'Right Front',
    'cal.lr': 'Left Rear',
    'cal.rr': 'Right Rear',
    'cal.hip': 'Hip (Roll)',
    'cal.thigh': 'Thigh (Pitch)',
    'cal.calf': 'Calf (Knee)',
    'cal.loaded': 'Calibration page loaded',
    'cal.saved': 'Profile saved: ',
    'cal.loaded_profile': 'Profile loaded: ',
    'cal.reset_confirm': 'Reset all calibration to zero?',
    'cal.reset_done': 'Calibration reset',
    'cal.applied': 'Calibration applied to robot',

    // Common
    'common.ws': 'WebSocket',
    'common.estop': 'E-Stop',
    'common.ros': 'ROS2',
    'common.servos': 'Servos',
  }
};

// Get current language
function getLang() {
  return localStorage.getItem('robot_dog_lang') || 'ru';
}

function setLang(lang) {
  localStorage.setItem('robot_dog_lang', lang);
  document.documentElement.lang = lang;
  applyTranslations();
  updateLangSwitcher();
}

// Translate a key
function t(key) {
  const lang = getLang();
  return (I18N[lang] && I18N[lang][key]) || (I18N['en'] && I18N['en'][key]) || key;
}

// Apply translations to all elements with data-i18n attribute
function applyTranslations() {
  document.querySelectorAll('[data-i18n]').forEach(el => {
    const key = el.getAttribute('data-i18n');
    el.textContent = t(key);
  });
  document.querySelectorAll('[data-i18n-placeholder]').forEach(el => {
    const key = el.getAttribute('data-i18n-placeholder');
    el.placeholder = t(key);
  });
}

// Build language switcher in nav
function initLangSwitcher() {
  const nav = document.querySelector('nav');
  if (!nav || document.getElementById('lang-switcher')) return;

  const sw = document.createElement('span');
  sw.id = 'lang-switcher';
  sw.style.cssText = 'float:right;display:flex;gap:2px;margin-left:auto;';
  sw.innerHTML = `
    <button class="lang-btn" data-lang="ru" onclick="setLang('ru')" style="padding:3px 8px;border:1px solid var(--border);background:var(--bg-card);color:var(--text);border-radius:4px;cursor:pointer;font-size:.8em">RU</button>
    <button class="lang-btn" data-lang="en" onclick="setLang('en')" style="padding:3px 8px;border:1px solid var(--border);background:var(--bg-card);color:var(--text);border-radius:4px;cursor:pointer;font-size:.8em">EN</button>
  `;
  nav.appendChild(sw);
  updateLangSwitcher();
}

function updateLangSwitcher() {
  const lang = getLang();
  document.querySelectorAll('.lang-btn').forEach(btn => {
    const active = btn.getAttribute('data-lang') === lang;
    btn.style.background = active ? 'var(--accent)' : 'var(--bg-card)';
    btn.style.color = active ? '#fff' : 'var(--text)';
    btn.style.fontWeight = active ? 'bold' : 'normal';
  });
}

// Init on DOM ready
document.addEventListener('DOMContentLoaded', () => {
  initLangSwitcher();
  applyTranslations();
  document.documentElement.lang = getLang();
});
