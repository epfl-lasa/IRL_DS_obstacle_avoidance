% Append a test series.
function series_result = appendseries(a,s,r,series_result,filename)

% Check if file exists.
if exist(filename,'file'),
    load(filename);
    series_result{s,a,r} = test_result;
else
    % Load a dummy.
    fprintf(1,'missing %i %i %i\n',a,s,r);
    if r ~= 1,
        series_result{s,a,r} = series_result{s,a,r-1};
    elseif s ~= 1,
        series_result{s,a,r} = series_result{s-1,a,r};
    else
        series_result{s,a,r} = series_result{s,a-1,r};
    end;
    series_result{s,a,r}.test_metrics = series_result{s,a,r}.test_metrics*NaN;
end;
