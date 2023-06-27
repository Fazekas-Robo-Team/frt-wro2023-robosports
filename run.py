#!/usr/bin/env python3

import sys
import os
import paramiko
import threading

from config import Ferenc, Viktor

ferenc = False
viktor = False

class SFTPClient(paramiko.SFTPClient):
    def put_dir(self, source, target):
        ''' Uploads the contents of the source directory to the target path. The
            target directory needs to exists. All subdirectories in source are 
            created under target.
        '''
        for item in os.listdir(source):
            if os.path.isfile(os.path.join(source, item)):
                print(f"{os.path.join(source, item)} -> {target}/{item}")
                self.put(os.path.join(source, item), '%s/%s' % (target, item))
            else:
                self.mkdir('%s/%s' % (target, item), ignore_existing=True)
                self.put_dir(os.path.join(source, item), '%s/%s' % (target, item))

    def mkdir(self, path, mode = 511, ignore_existing = False):
        ''' Augments mkdir by adding an option to not fail if the folder exists  '''
        try:
            super(SFTPClient, self).mkdir(path, mode)
        except IOError:
            if ignore_existing:
                pass
            else:
                raise

    def sync_dir(self, source, target):
        print("dir", source, target)
        remote = self.listdir(target)
        for item in os.listdir(source):
            if os.path.isfile(os.path.join(source, item)):
                src_path = os.path.join(source, item)
                dst_path = '%s/%s' % (target, item)
                if (not item in remote) or (self.stat(dst_path).st_mtime < os.stat(src_path).st_mtime):
                    print(f"{src_path} -> {dst_path}")
                    self.put(src_path, dst_path)
            else:
                self.mkdir('%s/%s' % (target, item), ignore_existing=True)
                self.sync_dir(os.path.join(source, item), '%s/%s' % (target, item))

def sync_robot(robot):
    transport = paramiko.Transport(robot.host, robot.port)
    transport.connect(username=robot.username, password=robot.password)
    sftp = SFTPClient.from_transport(transport)

    sftp.sync_dir("bin", "/home/robot/bin")

try:
    if sys.argv[1].lower() == "ferenc":
        ferenc = True
    elif sys.argv[1].lower() == "viktor":
        viktor = True
    elif sys.argv[1].lower() == "all":
        ferenc = True
        viktor = True

    if ferenc:
        os.system("make clean")
        os.system("FRT_ROBOT_ID=ferenc make")

        sync_robot(Ferenc)

    if viktor:
        os.system("make clean")
        os.system("FRT_ROBOT_ID=viktor make")

        sync_robot(Viktor)

    robots = []
    if ferenc: robots.append(Ferenc)
    if viktor: robots.append(Viktor)

    for r in robots: print(r)

    sshs = []
    for robot in robots:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(robot.host, robot.port, robot.username, robot.password, look_for_keys = False)
        sshs.append(ssh)

    for s in sshs: print(s)

    def run(ssh):
        stdin, stdout, stderr = ssh.exec_command("brickrun --redirect bash /home/robot/bin/start.sh")
        stdin.close()
        while line := stdout.readline():
            print(line, end = "")

    threads = []
    for ssh in sshs:
        t = threading.Thread(target = run, args = (ssh,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()



except IndexError:
    pass