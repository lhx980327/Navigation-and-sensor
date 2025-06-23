function y = convolve2(x, m, shape, tol)

% Deal with optional arguments
error(nargchk(2,4,nargin));
if nargin < 3
    shape = 'full';    % shape default as for CONV2
    tol = 0;
elseif nargin < 4
    if isnumeric(shape)
        tol = shape;
        shape = 'full';
    else
        tol = 0;
    end
end;

% Set up to do the wrap operation, not handled by conv2
if strcmp(shape, 'wrap')
    x = wraparound(x, m);
    shape = 'valid';
end

% do the convolution itself
y = doconv(x, m, shape, tol);

%-----------------------------------------------------------------------

function y = doconv(x, m, shape, tol);
% Carry out convolution
[mx, nx] = size(x);
[mm, nm] = size(m);

% If the mask is bigger than the input, or it is 1-D already,
% just let CONV2 handle it.
if mm > mx | nm > nx | mm == 1 | nm == 1
    y = conv2(x, m, shape);
else
    % Get svd of mask
    if mm < nm; m = m'; end        % svd(..,0) wants m > n
    [u,s,v] = svd(m, 0);
    s = diag(s);
    rank = trank(m, s, tol);
    if rank*(mm+nm) < mm*nm         % take advantage of low rank
        if mm < nm;  t = u; u = v; v = t; end  % reverse earlier transpose
        vp = v';
        % For some reason, CONV2(H,C,X) is very slow, so use the normal call
        y = conv2(conv2(x, u(:,1)*s(1), shape), vp(1,:), shape);
        for r = 2:rank
            y = y + conv2(conv2(x, u(:,r)*s(r), shape), vp(r,:), shape);
        end
    else
        if mm < nm; m = m'; end     % reverse earlier transpose
        y = conv2(x, m, shape);
    end
end

%-----------------------------------------------------------------------

function r = trank(m, s, tol)
% Approximate rank function - returns rank of matrix that fits given
% matrix to within given relative rms error. Expects original matrix
% and vector of singular values.
if tol < 0 | tol > 1
    error('Tolerance must be in range 0 to 1');
end
if tol == 0             % return estimate of actual rank
    tol = length(m) * max(s) * eps;
    r = sum(s > tol);
else
    ss = s .* s;
    t = (1 - tol) * sum(ss);
    r = 0;
    sm = 0;
    while sm < t
        r = r + 1;
        sm = sm + ss(r);
    end
end

%-----------------------------------------------------------------------

function y = wraparound(x, m)
% Extend x so as to wrap around on both axes, sufficient to allow a
% "valid" convolution with m to return the cyclical convolution.
% We assume mask origin near centre of mask for compatibility with
% "same" option.
[mx, nx] = size(x);
[mm, nm] = size(m);
if mm > mx | nm > nx
    error('Mask does not fit inside array')
end

mo = floor((1+mm)/2); no = floor((1+nm)/2);  % reflected mask origin
ml = mo-1;            nl = no-1;             % mask left/above origin
mr = mm-mo;           nr = nm-no;            % mask right/below origin
me = mx-ml+1;         ne = nx-nl+1;          % reflected margin in input
mt = mx+ml;           nt = nx+nl;            % top of image in output
my = mx+mm-1;         ny = nx+nm-1;          % output size

y = zeros(my, ny);
y(mo:mt, no:nt) = x;      % central region
if ml > 0
    y(1:ml, no:nt) = x(me:mx, :);                   % top side
    if nl > 0
        y(1:ml, 1:nl) = x(me:mx, ne:nx);            % top left corner
    end
    if nr > 0
        y(1:ml, nt+1:ny) = x(me:mx, 1:nr);          % top right corner
    end
end
if mr > 0
    y(mt+1:my, no:nt) = x(1:mr, :);                 % bottom side
    if nl > 0
        y(mt+1:my, 1:nl) = x(1:mr, ne:nx);          % bottom left corner
    end
    if nr > 0
        y(mt+1:my, nt+1:ny) = x(1:mr, 1:nr);        % bottom right corner
    end
end
if nl > 0
    y(mo:mt, 1:nl) = x(:, ne:nx);                   % left side
end
if nr > 0
    y(mo:mt, nt+1:ny) = x(:, 1:nr);                 % right side
end
