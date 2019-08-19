% Evaluate the test metrics on example and test paths obtained from true
% reward and IRL reward.
function test_metrics = evaluatematrics(trgnd,trirl,tsgnd,tsirl,rgnd,rirl,irl_result)

% See metricnames.m for which index corresponds to which metric.
test_metrics = zeros(1,8);

% Compute reward sums.
trgndr = 0;
tsgndr = 0;
trirlr = 0;
tsirlr = 0;
for i=1:length(trgnd),
    trgndr = trgndr + trgnd{i}.r;
    trirlr = trirlr + trirl{i}.r;
end;
for i=1:length(tsgnd),
    tsgndr = tsgndr + tsgnd{i}.r;
    tsirlr = tsirlr + tsirl{i}.r;
end;

% Compute metrics.
test_metrics(1) = trgndr;
test_metrics(2) = trirlr;
test_metrics(3) = trgndr - trirlr;
test_metrics(4) = abs(trgndr - trirlr)/abs(trgndr);

test_metrics(5) = tsgndr;
test_metrics(6) = tsirlr;
test_metrics(7) = tsgndr - tsirlr;
test_metrics(8) = abs(tsgndr - tsirlr)/abs(tsgndr);

% Add timing metric.
test_metrics = [irl_result.total_time test_metrics];
