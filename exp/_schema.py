import sqlite3

paths = [
    '/home/ywj/slam_data/livox_record_noshell/livox_record_noshell_0.db3',
    '/home/ywj/slam_data/livox_record_noshell/livox_record1_0.db3',
]
for p in paths:
    c = sqlite3.connect(p)
    tabs = [r[0] for r in c.execute("select name from sqlite_master where type='table'")]
    print(p, tabs)
    for t in tabs:
        try:
            n = c.execute('select count(*) from "' + t + '"').fetchone()[0]
            print('   ', t, n)
        except Exception as e:
            print('   ', t, 'err', e)
    try:
        t0, t1, n = c.execute('select min(timestamp),max(timestamp),count(*) from messages').fetchone()
        print('    dur=%.1f s msgs=%d' % ((t1 - t0) / 1e9, n))
        for r in c.execute("select t.name, count(*) from messages m join topics t on t.id=m.topic_id group by t.name"):
            print('    ', r)
    except Exception as e:
        print('    messages err', e)
