import sqlite3

p = '/home/ywj/slam_data/Retail_Street/Retail_Street.db3'
c = sqlite3.connect(p)
tabs = [r[0] for r in c.execute("select name from sqlite_master where type='table'")]
print('tables:', tabs)
for r in c.execute('select id,name,type,serialization_format,offered_qos_profiles from topics'):
    print(r[0], r[1], r[2], r[3], (r[4] or '')[:120].replace('\n', ' '))
t0, t1, n = c.execute('select min(timestamp),max(timestamp),count(*) from messages').fetchone()
print('messages=%d dur=%.1f s' % (n, (t1 - t0) / 1e9))
for r in c.execute('select t.name, count(*) from messages m join topics t on t.id=m.topic_id group by t.name'):
    print('  ', r[0], r[1])
