load('all_val');
[b,a] = tfdata(x_tf_bad);
[A1,B1,C,D] = tf2ss(cell2mat(b),cell2mat(a));

[b,a] = tfdata(y_tf);
[A2,B2,C,D] = tf2ss(cell2mat(b),cell2mat(a));

[b,a] = tfdata(height_tf);
[A3,B3,C,D] = tf2ss(cell2mat(b),cell2mat(a));

A1
A2
A3