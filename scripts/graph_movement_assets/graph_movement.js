function range_start(array, value)
{
    var count = array.length;
    var first = 0;

    while (count > 0) {
        var elem = first; 
        var step = Math.floor(count / 2); 
        elem += step;
        if (array[elem] < value) {
            first = elem + 1;
            count -= step + 1; 
        }
        else { 
            count = step;
        }
    }
    return first < array.length ? first : array.length-1;
}

function range_end(array, value)
{
    var count = array.length
    var first = 0;

    while (count > 0) {
        var elem = first; 
        var step = Math.floor(count / 2); 
        elem += step;
        if (array[elem] <= value) {
            first = elem + 1;
            count -= step + 1; 
        }
        else { 
            count = step;
        }
    }
    return first < array.length ? first : array.length-1;
}

function zoom_trace_y(xvals, yvals, start, end)
{
    if (xvals.length == 0) {
        return [-100, 100];
    }
    var i_low = range_start(xvals, start);
    var i_high = range_end(xvals, end);
    var num_steps = xvals.length;
    if (i_low >= num_steps) {
        var range_low = yvals[array.length - 1];
        var range_high = range_low;
    } else if (i_low == i_high) {
        var range_low = yvals[i_low];
        var range_high = range_low;
    } else {
        var array_range = yvals.slice(i_low, i_high-1);
        var range_low = Math.min(...array_range);
        var range_high = Math.max(...array_range);
    }
    var diff = range_high - range_low;
    var margin = diff * 0.1;
    range_high += margin;
    range_low -= margin;
    return [range_low, range_high];
}

function zoom_figure_y(fig)
{
    fig = {...fig}
    x_range = fig.layout.xaxis.range
    fig.data.forEach((element, i) => {
       var y_range = zoom_trace_y(element.x, element.y, x_range[0], x_range[1]);
       var axis_name = "yaxis"
       if (i > 0) {
           axis_name = axis_name + (i+1);
       }
       fig.layout[axis_name].range = y_range
       fig.layout[axis_name].autorange = false;
    });
    return fig
}