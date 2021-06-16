function v = generate_noise(t)
    v = zeros(4,1);
    
    % noise parameters
    acc_noise_freq   = [2; 5];
    acc_noise_mag    = [0.1; 0.1];
    acc_noise_offset = [0; 0];

    % continuous noise
    v(2) = acc_noise_mag(1)*sin(acc_noise_freq(1)*t) + acc_noise_offset(1);
    v(4) = acc_noise_mag(2)*sin(acc_noise_freq(2)*t) + acc_noise_offset(2);

end