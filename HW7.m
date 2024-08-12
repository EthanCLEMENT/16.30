% Define the transfer functions
s = tf('s');
G = [0 tf([3 0],[1 1 10]); tf([1 1],[1 5]), tf(2,[1 6])];

% Define the range of frequencies (logarithmically spaced)
frequencies = logspace(-2, 2, 1000); % from 0.01 to 100 rad/s

% Initialize variable to store the maximum singular value
max_singular_value = 0;

% Loop through each frequency
for i = 1:length(frequencies)
    % Compute the frequency response at the current frequency
    freq_response = evalfr(G, 1j*frequencies(i));
    
    % Compute the singular values of the frequency response matrix
    singular_values = svd(freq_response);
    
    % Update the maximum singular value
    max_singular_value = max(max_singular_value, max(singular_values));
end

% Display the largest singular value
disp('Largest singular value across frequencies:');
disp(max_singular_value);
