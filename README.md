# IT4663_miniprj
Min Max Vehicle Routing Postman Collecting packages

Có N điểm 1, 2, …, N cần thu gom bưu kiện và K bưu tá xuất phát từ bưu điện (điểm 0).
Biết d(i,j) là khoảng cách từ điểm i đến điểm j, với i,j = 0,1,..., N 

Cần xây dựng phương án thu gom cho K bưu tá, xác định mỗi bưu tá thu gom nhưng điểm nào và theo thứ tự nào sao cho quãng đường dài nhất của bưu tá phải ngắn nhất.

A solution is K routes for K vehicles, each route k is represented by x[1], x[2], . . ., x[lk]  in which x[1] = 0 (the depot), and x[2], x[3], . . , x[lk] are pickup points.

Input 

Line 1: contains N and K (1 <= N <= 1000, 1 <= K <= 100)

Line i+1 (i = 0,…,N): contains the ith row of the distance matrix d

Output

Line 1: contains K

Line 2*k (k = 1, . . ., K): contains lk

Line 2*k+1 (k = 1, . . ., K): contains lk  integers x[1], x[2], . . . , x[lk] (elements are separated by a SPACE character)