"""Shared pieces of the HTML report builders: video files, posters, tables.

Videos are never embedded: each page references videos/<name>.mp4 and
posters/<name>.jpg next to it (report/ in the repository)."""
import os
import shutil
import subprocess

try:
    import cv2
    import imageio_ffmpeg
except ImportError as e:  # pragma: no cover
    raise SystemExit(f'{e}: pip install opencv-python-headless imageio-ffmpeg')

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, 'template.html')
DATA = os.path.join(HERE, 'data')


def encode(src, dst, crf=32):
    """Raw render -> 960x540 H.264 MP4 (small enough for git, plays everywhere)."""
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    subprocess.run([ff, '-v', 'error', '-y', '-i', src, '-c:v', 'libx264', '-preset', 'slow', '-crf', str(crf),
                    '-vf', 'scale=960:540:flags=lanczos', '-profile:v', 'main', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', '-an', dst], check=True)


def video(out_dir, name, raw_dir=None, crf=32, poster_at=0.6, raw_name=None):
    """(video src, poster src) relative to the page.

    With raw_dir holding <raw_name or name>.mp4 (a fresh render) it is re-encoded into
    out_dir/videos when newer; otherwise the video already in out_dir/videos
    is used as it is. The poster is a frame at poster_at of the clip."""
    vdir, pdir = os.path.join(out_dir, 'videos'), os.path.join(out_dir, 'posters')
    os.makedirs(vdir, exist_ok=True)
    os.makedirs(pdir, exist_ok=True)
    dst, poster = os.path.join(vdir, f'{name}.mp4'), os.path.join(pdir, f'{name}.jpg')
    raw = os.path.join(raw_dir, f'{raw_name or name}.mp4') if raw_dir else None
    fresh = False
    if raw and os.path.exists(raw) and (not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(raw)):
        encode(raw, dst, crf)
        fresh = True
    if not os.path.exists(dst):
        raise SystemExit(f'missing video {dst} (render it, then pass its folder with --raw)')
    if fresh or not os.path.exists(poster):
        c = cv2.VideoCapture(dst)
        n = int(c.get(cv2.CAP_PROP_FRAME_COUNT))
        c.set(cv2.CAP_PROP_POS_FRAMES, int(n * poster_at))
        ok, f = c.read()
        if not ok:
            raise SystemExit(f'cannot read {dst}')
        cv2.imwrite(poster, cv2.resize(f, (640, 360)), [cv2.IMWRITE_JPEG_QUALITY, 70])
    return f'videos/{name}.mp4', f'posters/{name}.jpg'


def copy_if_newer(src, dst):
    if not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(src):
        shutil.copyfile(src, dst)


def cell(v):
    if v is None:
        return '<td></td>'
    txt, cls = v
    return f'<td class="num r-{cls}">{txt}</td>'


def table(head, rows):
    h = ''.join(f'<th class="num">{x}</th>' if i else f'<th>{x}</th>' for i, x in enumerate(head))
    b = ''.join('<tr><td>' + r[0] + '</td>' + ''.join(cell(c) for c in r[1:]) + '</tr>' for r in rows)
    return f'<div class="tbl-wrap"><table><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>'


def P(x):
    return (x, 'ok')


def W(x):
    return (x, 'warn')


def F(x):
    return (x, 'bad')


X = ('упал', 'fall')


def template_css():
    t = open(TEMPLATE, encoding='utf-8').read()
    return t.split('<style>')[1].split('</style>')[0]


def template_script():
    t = open(TEMPLATE, encoding='utf-8').read()
    return t[t.rfind('<script>'):t.rfind('</script>') + len('</script>')]
