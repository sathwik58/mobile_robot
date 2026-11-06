#!/usr/bin/env python3
import glob
import os
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

WS = os.path.expanduser('~/mobile_robot')
WORLDS = os.path.join(WS, 'src', 'amr', 'urdf')
MAPS = os.path.join(WS, 'src', 'amr_nav', 'maps')

TASKS = {
    'Make a map (SLAM + teleop)': ('amr_nav', 'slam.launch.py', 'world1.usd', False),
    'Part A - Navigation on saved map': ('amr_nav', 'nav.launch.py', 'world1.usd', True),
    'Part B - Navigate while mapping (maze)': ('amr_nav', 'slam_nav.launch.py', 'world_maze.usd', False),
    'Part C - Corridor lane coverage': ('amr_nav', 'coverage.launch.py', 'world_corridor.usd', False),
    'Robot only (Isaac + RViz + teleop)': ('amr', 'isaac.launch.py', '', False),
}


class AmrGui:
    def __init__(self, root):
        self.root = root
        self.proc = None
        self.lines = queue.Queue()
        root.title('AMR launcher')
        root.geometry('900x620')
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        top = ttk.Frame(root, padding=10)
        top.pack(fill='x')
        ttk.Label(top, text='Task').grid(row=0, column=0, sticky='w')
        self.task = tk.StringVar(value=list(TASKS)[0])
        box = ttk.Combobox(top, textvariable=self.task, values=list(TASKS), state='readonly', width=42)
        box.grid(row=0, column=1, sticky='w', padx=6)
        box.bind('<<ComboboxSelected>>', lambda e: self.on_task())

        ttk.Label(top, text='World').grid(row=1, column=0, sticky='w', pady=4)
        self.world = tk.StringVar()
        self.world_box = ttk.Combobox(top, textvariable=self.world, state='readonly', width=42)
        self.world_box.grid(row=1, column=1, sticky='w', padx=6)

        ttk.Label(top, text='Map (Part A)').grid(row=2, column=0, sticky='w')
        self.map = tk.StringVar()
        self.map_box = ttk.Combobox(top, textvariable=self.map, state='readonly', width=42)
        self.map_box.grid(row=2, column=1, sticky='w', padx=6)

        opts = ttk.Frame(top)
        opts.grid(row=0, column=2, rowspan=3, sticky='nw', padx=20)
        self.headless = tk.BooleanVar(value=False)
        self.rviz = tk.BooleanVar(value=True)
        ttk.Checkbutton(opts, text='Isaac headless (no window)', variable=self.headless).pack(anchor='w')
        ttk.Checkbutton(opts, text='Open RViz', variable=self.rviz).pack(anchor='w')

        btns = ttk.Frame(root, padding=(10, 0))
        btns.pack(fill='x')
        self.start_btn = ttk.Button(btns, text='Start', command=self.start)
        self.start_btn.pack(side='left')
        self.stop_btn = ttk.Button(btns, text='Stop', command=self.stop, state='disabled')
        self.stop_btn.pack(side='left', padx=6)
        ttk.Button(btns, text='Teleop keyboard', command=self.teleop).pack(side='left', padx=6)
        ttk.Separator(btns, orient='vertical').pack(side='left', fill='y', padx=10)
        ttk.Label(btns, text='Map name').pack(side='left')
        self.map_name = tk.StringVar(value='world1')
        ttk.Entry(btns, textvariable=self.map_name, width=16).pack(side='left', padx=4)
        ttk.Button(btns, text='Save map', command=self.save_map).pack(side='left')

        self.status = tk.StringVar(value='idle')
        ttk.Label(root, textvariable=self.status, padding=(10, 6), foreground='#0a6').pack(fill='x')
        frame = ttk.Frame(root, padding=(10, 0, 10, 10))
        frame.pack(fill='both', expand=True)
        self.log = tk.Text(frame, wrap='none', bg='#111', fg='#ddd', font=('Monospace', 9))
        scroll = ttk.Scrollbar(frame, command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.log.pack(side='left', fill='both', expand=True)

        self.refresh_lists()
        self.on_task()
        self.root.after(100, self.pump)

    def refresh_lists(self):
        worlds = sorted(os.path.basename(p) for p in glob.glob(os.path.join(WORLDS, '*.usd'))
                        if os.path.basename(p) != 'amr.usd')
        self.world_box['values'] = ['(none - ground only)'] + worlds
        maps = sorted(os.path.basename(p) for p in glob.glob(os.path.join(MAPS, '*.yaml'))
                      if not p.endswith('_isaac.yaml'))
        self.map_box['values'] = maps

    def on_task(self):
        self.refresh_lists()
        _, _, world, needs_map = TASKS[self.task.get()]
        self.world.set(world if world in self.world_box['values'] else self.world_box['values'][0])
        if needs_map:
            stem = os.path.splitext(world)[0] + '.yaml'
            maps = list(self.map_box['values'])
            self.map.set(stem if stem in maps else (maps[0] if maps else ''))
            self.map_box.configure(state='readonly')
        else:
            self.map.set('')
            self.map_box.configure(state='disabled')
        self.map_name.set(os.path.splitext(world)[0] if world else 'map')

    def write(self, text):
        self.log.insert('end', text)
        self.log.see('end')

    def run(self, cmd, label):
        self.write(f'\n$ {" ".join(cmd)}\n')
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1, start_new_session=True)
        threading.Thread(target=self.reader, args=(self.proc,), daemon=True).start()
        self.status.set(f'running: {label}')
        self.start_btn.configure(state='disabled')
        self.stop_btn.configure(state='normal')

    def reader(self, proc):
        for line in proc.stdout:
            self.lines.put(line)
        self.lines.put(None)

    def pump(self):
        try:
            while True:
                line = self.lines.get_nowait()
                if line is None:
                    self.on_exit()
                else:
                    self.write(line)
        except queue.Empty:
            pass
        self.root.after(100, self.pump)

    def on_exit(self):
        if self.proc and self.proc.poll() is not None:
            self.status.set(f'stopped (exit code {self.proc.returncode})')
            self.proc = None
            self.start_btn.configure(state='normal')
            self.stop_btn.configure(state='disabled')

    def start(self):
        if self.proc:
            return
        pkg, launch, _, needs_map = TASKS[self.task.get()]
        cmd = ['ros2', 'launch', pkg, launch,
               f'headless:={str(self.headless.get()).lower()}', f'rviz:={str(self.rviz.get()).lower()}']
        world = self.world.get()
        if world and not world.startswith('('):
            cmd.append(f'world:={os.path.join(WORLDS, world)}')
        elif launch != 'isaac.launch.py':
            self.write('\nthis task needs a world\n')
            return
        if needs_map:
            if not self.map.get():
                self.write('\nno saved map - make one first with "Make a map"\n')
                return
            cmd.append(f'map:={os.path.join(MAPS, self.map.get())}')
        self.run(cmd, self.task.get())

    def stop(self):
        if not self.proc:
            return
        self.status.set('stopping ...')
        proc = self.proc

        def kill():
            for sig, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 10), (signal.SIGKILL, 0)):
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    return
                t0 = time.time()
                while time.time() - t0 < wait:
                    if proc.poll() is not None:
                        return
                    time.sleep(0.2)

        threading.Thread(target=kill, daemon=True).start()

    def teleop(self):
        subprocess.Popen(['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard',
                          'teleop_twist_keyboard'], start_new_session=True)

    def save_map(self):
        name = self.map_name.get().strip() or 'map'
        out = os.path.join(MAPS, name)
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', out,
               '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=20.0']
        self.write(f'\n$ {" ".join(cmd)}\n')

        def job():
            r = subprocess.run(cmd, capture_output=True, text=True)
            ok = 'saved successfully' in (r.stdout + r.stderr)
            self.lines.put(f'map save {"OK -> " + out + ".yaml" if ok else "FAILED"}\n')
            if not ok:
                self.lines.put(r.stdout[-800:] + r.stderr[-800:])
            self.root.after(0, self.refresh_lists)

        threading.Thread(target=job, daemon=True).start()

    def on_close(self):
        if self.proc:
            self.stop()
            self.root.after(1500, self.on_close)
            return
        self.root.destroy()


def main():
    root = tk.Tk()
    AmrGui(root)
    root.mainloop()


if __name__ == '__main__':
    main()
