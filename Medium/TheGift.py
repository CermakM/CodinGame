# Ideal budget distribution / The Gift task - Python3

N, price = map(int, (input(), input()))
budgets = sorted([ int(input()) for i in range(N)]);

# Check if it is possible to buy a present
if (sum(budgets) < price): 
    print("IMPOSSIBLE");
    exit();

res = "";
for p in range(N):
    charge = min(price // (N - p), budgets[p])
    price -= charge
    res += (str(charge) + '\n')

print(res);

