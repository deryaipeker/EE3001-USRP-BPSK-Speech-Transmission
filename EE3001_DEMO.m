1%% ========================================================================
% EE3001 FULL PROJECT - ONE MATLAB FILE ONLY
% Loads ONLY x_corrupt.mat. Everything else is inside this file.
% Demo outputs: BER, SNR, constellation diagrams, TX waveform before
% transmission, RX waveform before demodulation, reconstructed speech.
% ========================================================================
clear; close all; clc;

fprintf('\n========== EE3001 FULL PROJECT: DSP + BPSK + USRP WIRELESS ==========%s', newline);
outDir = fullfile(pwd,'outputs');
if ~exist(outDir,'dir'), mkdir(outDir); end

%% -------------------- Configuration --------------------
cfg.fs_input  = 48000;
cfg.fs_speech = 8000;
cfg.frame_length = 160;      % 20 ms at 8 kHz
cfg.frame_shift  = 80;       % 10 ms hop
cfg.pre_emph = 0.97;
cfg.lpc_order = 14;
cfg.pitch_min = round(cfg.fs_speech/300);
cfg.pitch_max = round(cfg.fs_speech/60);
cfg.lsf_bit_allocation = [6 6 6 6 5 5 5 5 5 5 5 5 5 5];
cfg.gain_bits = 7;
cfg.pitch_bits = 7;
cfg.vuv_bits = 1;
cfg.bits_per_frame = cfg.vuv_bits + cfg.pitch_bits + cfg.gain_bits + sum(cfg.lsf_bit_allocation);
% Telecom settings for antenna/wireless mode
cfg.symbol_rate = 10e3;
cfg.samples_per_symbol = 20;
cfg.usrp_sample_rate = cfg.symbol_rate * cfg.samples_per_symbol; % 200 kHz
cfg.rolloff = 0.35;
cfg.filter_span = 8;
cfg.preamble_bits = repmat([1;0;1;0;1;1;0;0],64,1); % 512 bits

% Forward error correction (ECC) for the speech-parameter bit stream.
% "conv"    = rate-1/2 constraint-length-7 convolutional code + Viterbi (default)
% "hamming" = Hamming(7,4) block code
% "none"    = no coding (identical behaviour to the original project)
cfg.ecc_type     = "conv";
cfg.conv_trellis = poly2trellis(7,[171 133]); % standard rate-1/2 K=7 code
cfg.conv_tblen   = 30;                          % Viterbi traceback depth

cfg.usrp_platform = "B200";
cfg.usrp_serial = "";
cfg.master_clock_rate = 20e6;
cfg.interp_factor = cfg.master_clock_rate/cfg.usrp_sample_rate;
cfg.decim_factor = cfg.interp_factor;
cfg.center_freq = 915e6;
cfg.tx_gain = 60;
cfg.rx_gain = 45;
cfg.rx_frame_size = 4096;
cfg.num_packet_repeats = 50;
cfg.extra_rx_chunks = 80;

fprintf('Speech Fs: %d Hz -> %d Hz | LPC order: %d | %d bits/frame\n', cfg.fs_input, cfg.fs_speech, cfg.lpc_order, cfg.bits_per_frame);
fprintf('BPSK: %.0f sym/s, %d sps, %.0f Hz sample rate | fc = %.3f MHz\n', cfg.symbol_rate, cfg.samples_per_symbol, cfg.usrp_sample_rate, cfg.center_freq/1e6);
fprintf('USRP: %s serial %s | TX gain %.1f | RX gain %.1f\n', cfg.usrp_platform, cfg.usrp_serial, cfg.tx_gain, cfg.rx_gain);

%% -------------------- Load only x_corrupt.mat --------------------
if ~exist('x_corrupt.mat','file')
    error('x_corrupt.mat not found. Put it in the same folder as this MATLAB file.');
end
S = load('x_corrupt.mat');   % The only external file loaded
fn = fieldnames(S);
x_corrupt = double(S.(fn{1}));
x_corrupt = x_corrupt(:);

%% -------------------- DSP transmitter --------------------
fprintf('\n[1] DSP transmitter: filtering, resampling, LPC/LSF, quantization\n');
speech_8k = signal_preparation_local(x_corrupt,cfg);
params = lpc_lsf_analysis_local(speech_8k,cfg);
[tx_bits, q_params] = quantize_parameters_local(params,cfg); %#ok<NASGU>

fid = fopen('tx_bit_stream.txt','w'); fprintf(fid,'%d',tx_bits); fclose(fid);
audiowrite(fullfile(outDir,'speech_8k.wav'),speech_8k,cfg.fs_speech);

% Channel coding: protect the speech-parameter bits with ECC before BPSK
tx_coded = ecc_encode_local(tx_bits,cfg);
fprintf('[ecc] %s coding: %d message bits -> %d coded bits (rate %.2f)\n', ...
    upper(string(cfg.ecc_type)), numel(tx_bits), numel(tx_coded), numel(tx_bits)/max(1,numel(tx_coded)));

raw_rate = cfg.fs_speech*16;
encoded_rate = cfg.bits_per_frame*(cfg.fs_speech/cfg.frame_shift);
fprintf('Raw PCM bit rate: %.1f kbps | Encoded rate: %.1f kbps | Compression: %.2fx\n', raw_rate/1000, encoded_rate/1000, raw_rate/encoded_rate);

% Local DSP reconstruction, before telecom
local_params = dequantize_parameters_local(tx_bits,params.num_frames,cfg);
rng(1);
speech_local = speech_synthesis_local(local_params,cfg);
audiowrite(fullfile(outDir,'reconstructed_local_DSP.wav'),speech_local,cfg.fs_speech);

%% -------------------- BPSK modulation --------------------
fprintf('\n[2] BPSK modulation and pulse shaping\n');
[tx_packet, rrc_filter, tx_full_bits, tx_symbols] = bpsk_modulate_local(tx_coded,cfg); %#ok<ASGLU>

%% -------------------- Software simulation --------------------
fprintf('\n[3] Software BPSK simulation: BER, SNR, constellation\n');
[rx_ideal, ber_ideal, diag_ideal] = bpsk_demodulate_local(tx_packet,tx_coded,cfg.preamble_bits,cfg.samples_per_symbol,rrc_filter); %#ok<ASGLU>
fprintf('Ideal-channel coded BER = %.6g\n', ber_ideal);

snr_values = [0 3 5 8 10 15 20 30];
ber_values     = zeros(size(snr_values));   % raw coded-bit BER (before ECC decoding)
ber_values_dec = zeros(size(snr_values));   % message BER after ECC decoding
diag_example = [];
for k = 1:numel(snr_values)
    rx_noisy = add_awgn_local(tx_packet,snr_values(k));
    [rx_coded_sim, ber_values(k), dtmp] = bpsk_demodulate_local(rx_noisy,tx_coded,cfg.preamble_bits,cfg.samples_per_symbol,rrc_filter);
    rx_msg_sim = ecc_decode_local(rx_coded_sim,numel(tx_bits),cfg);
    ber_values_dec(k) = sum(double(rx_msg_sim)~=double(tx_bits))/numel(tx_bits);
    if snr_values(k)==10
        diag_example = dtmp;
    end
end

make_pre_usrp_plots_local(x_corrupt,speech_8k,speech_local,params,tx_bits,tx_packet,tx_symbols,snr_values,ber_values,ber_values_dec,diag_example,cfg,outDir);

%% -------------------- Effect of channel SNR, with and without ECC --------------------
% Software-only study (no USRP): pass the BPSK packet through AWGN at a few SNR
% levels and reconstruct speech in two ways - WITH the ECC decoder and WITHOUT
% it (raw uncoded BPSK) - to show how channel noise corrupts the speech and how
% ECC protects it. Saves one wav per case + a side-by-side spectrogram figure.
fprintf('\n[3b] Reconstructed speech vs channel SNR, with and without ECC\n');
snr_demo = [-11 -9 -7];
tx_packet_unc = bpsk_modulate_local(tx_bits,cfg); % uncoded packet for the no-ECC branch
speech_ecc   = cell(numel(snr_demo),1);
speech_noecc = cell(numel(snr_demo),1);
rng(1);  % reproducible channel noise so the demo always shows the same result
for k = 1:numel(snr_demo)
    s = snr_demo(k);
    % --- with ECC: coded packet -> demod -> Viterbi decode ---
    try
        rc = bpsk_demodulate_local(add_awgn_local(tx_packet,s),tx_coded,cfg.preamble_bits,cfg.samples_per_symbol,rrc_filter);
        rb = ecc_decode_local(rc,length(tx_bits),cfg);
        ber_e = mean(double(rb)~=double(tx_bits));
        speech_ecc{k} = speech_synthesis_local(dequantize_parameters_local(rb,params.num_frames,cfg),cfg);
        audiowrite(fullfile(outDir,sprintf('reconstructed_snr%+03ddB_ecc.wav',s)),speech_ecc{k},cfg.fs_speech);
    catch ME
        ber_e = NaN; speech_ecc{k} = []; fprintf('   ECC branch skipped at %d dB (%s)\n',s,ME.message);
    end
    % --- without ECC: uncoded packet -> demod -> raw bits ---
    try
        rb2 = bpsk_demodulate_local(add_awgn_local(tx_packet_unc,s),tx_bits,cfg.preamble_bits,cfg.samples_per_symbol,rrc_filter);
        ber_n = mean(double(rb2)~=double(tx_bits));
        speech_noecc{k} = speech_synthesis_local(dequantize_parameters_local(rb2,params.num_frames,cfg),cfg);
        audiowrite(fullfile(outDir,sprintf('reconstructed_snr%+03ddB_noecc.wav',s)),speech_noecc{k},cfg.fs_speech);
    catch ME
        ber_n = NaN; speech_noecc{k} = []; fprintf('   no-ECC branch skipped at %d dB (%s)\n',s,ME.message);
    end
    fprintf('  SNR %+3d dB | message BER: no-ECC = %.4g , ECC = %.4g\n',s,ber_n,ber_e);
end
make_speech_quality_plot_local(speech_8k,speech_noecc,speech_ecc,snr_demo,cfg,outDir);

%% -------------------- Optional USRP wireless test --------------------
run_usrp = input('\nRun USRP wireless antenna test? 1=yes, 0=no: ');
if isempty(run_usrp) || run_usrp ~= 1
    fprintf('Stopped after DSP + BPSK simulation.\n');
    return;
end

fprintf('\n[4] USRP wireless TX/RX with antennas\n');
fprintf('Hardware: TX1/RX1 antenna ))) wireless channel ))) RX2 antenna\n');
fprintf('Keep antennas vertical and parallel. Do not touch them during capture.\n');

tx_signal = repmat(tx_packet,cfg.num_packet_repeats,1);
pad_len = mod(-length(tx_signal),cfg.rx_frame_size);
tx_padded = [tx_signal; complex(zeros(pad_len,1),0)];
num_chunks = length(tx_padded)/cfg.rx_frame_size;
total_chunks = num_chunks + cfg.extra_rx_chunks;
fprintf('Packet repeats: %d | TX chunks: %d | Extra RX chunks: %d\n', cfg.num_packet_repeats, num_chunks, cfg.extra_rx_chunks);

%% Create USRP transmitter and receiver objects
fprintf('\nCreating USRP TX/RX objects...\n');

% IMPORTANT FIX:
% Some MATLAB/USRP support-package versions do NOT allow SerialNum = "".
% Therefore, if cfg.usrp_serial is empty, we automatically detect the real
% serial number using findsdru/findsdr, then create TX/RX with that serial.

if strlength(string(cfg.usrp_serial)) == 0
    fprintf('No serial number specified. Searching for connected USRP...\n');
    cfg.usrp_serial = auto_detect_usrp_serial_local(cfg.usrp_platform);
    fprintf('Auto-detected USRP serial: %s\n', string(cfg.usrp_serial));
else
    fprintf('Using specified USRP serial: %s\n', string(cfg.usrp_serial));
end

% Always create the objects with a real non-empty serial number.
tx_usrp = comm.SDRuTransmitter( ...
    "Platform", cfg.usrp_platform, ...
    "SerialNum", cfg.usrp_serial, ...
    "CenterFrequency", cfg.center_freq, ...
    "Gain", cfg.tx_gain, ...
    "MasterClockRate", cfg.master_clock_rate, ...
    "InterpolationFactor", cfg.interp_factor);

rx_usrp = comm.SDRuReceiver( ...
    "Platform", cfg.usrp_platform, ...
    "SerialNum", cfg.usrp_serial, ...
    "CenterFrequency", cfg.center_freq, ...
    "Gain", cfg.rx_gain, ...
    "MasterClockRate", cfg.master_clock_rate, ...
    "DecimationFactor", cfg.decim_factor, ...
    "SamplesPerFrame", cfg.rx_frame_size, ...
    "OutputDataType", "double");

fprintf('Warming up USRP...\n');
zero_chunk = complex(zeros(cfg.rx_frame_size,1));
for k = 1:5
    tx_usrp(zero_chunk);
    rx_usrp();
end

rx_buffer = complex(zeros(total_chunks*cfg.rx_frame_size,1));
ptr = 0;
fprintf('Starting wireless capture...\n');
for k = 1:total_chunks
    if k <= num_chunks
        chunk = tx_padded((k-1)*cfg.rx_frame_size+1:k*cfg.rx_frame_size);
    else
        chunk = complex(zeros(cfg.rx_frame_size,1));
    end
    tx_usrp(chunk);
    [rx_chunk,len,overrun] = rx_usrp();
    if overrun
        warning('RX overrun at chunk %d',k);
    end
    if len > 0
        rx_buffer(ptr+1:ptr+len) = rx_chunk(1:len);
        ptr = ptr + len;
    end
    if mod(k,50)==0 || k==total_chunks
        fprintf('Processed chunk %d / %d\n',k,total_chunks);
    end
end
release(tx_usrp); release(rx_usrp);
rx_buffer = rx_buffer(1:ptr);
save(fullfile(outDir,'usrp_rx_buffer.mat'),'rx_buffer');

rx_rms = rms(abs(rx_buffer));
rx_peak = max(abs(rx_buffer));
fprintf('Collected %d RX samples.\n',length(rx_buffer));
fprintf('RX diagnostics: RMS = %.8f, peak = %.8f\n',rx_rms,rx_peak);

%% -------------------- Demodulation + reconstruction --------------------
fprintf('\n[5] BPSK demodulation and speech reconstruction\n');
[rx_coded, ber_coded, diag_usrp] = bpsk_demodulate_local(rx_buffer,tx_coded,cfg.preamble_bits,cfg.samples_per_symbol,rrc_filter);

% ECC decoding: recover the original speech-parameter bits from the coded stream
rx_bits   = ecc_decode_local(rx_coded,length(tx_bits),cfg);
ber_usrp  = sum(double(rx_bits)~=double(tx_bits))/length(tx_bits);
coded_errors = sum(double(rx_coded)~=double(tx_coded));
msg_errors   = sum(double(rx_bits)~=double(tx_bits));

fid = fopen('rx_bit_stream.txt','w'); fprintf(fid,'%d',rx_bits); fclose(fid);

fprintf('\n========== WIRELESS RESULT ==========\n');
fprintf('Coded channel BER = %.6g (%d / %d coded-bit errors before ECC)\n', ber_coded, coded_errors, length(tx_coded));
fprintf('Message BER (ECC) = %.6g (%d / %d errors after ECC)\n', ber_usrp, msg_errors, length(tx_bits));
fprintf('Estimated SNR   = %.2f dB\n', diag_usrp.snr_est_db);
fprintf('normCorr        = %.3f\n', diag_usrp.norm_corr);
fprintf('Preamble BER    = %.4f\n', diag_usrp.pre_ber);
fprintf('=====================================\n');

rx_params = dequantize_parameters_local(rx_bits,params.num_frames,cfg);
speech_rx = speech_synthesis_local(rx_params,cfg);

% Final enhancement for reconstructed LPC/LSF speech
speech_rx = highpass(speech_rx, 70, cfg.fs_speech);
speech_rx = lowpass(speech_rx, 3800, cfg.fs_speech);

% Presence boost for clearer speech around 1.2–3.2 kHz
[b_pres, a_pres] = butter(2, [1200 3200]/(cfg.fs_speech/2), 'bandpass');
presence = filter(b_pres, a_pres, speech_rx);
speech_rx = speech_rx + 0.25 * presence;

% Very light smoothing only
speech_rx = smoothdata(speech_rx, 'movmean', 2);

% Loudness boost with soft clipping
speech_rx = speech_rx / (max(abs(speech_rx)) + 1e-12);
speech_rx = 2.0 * speech_rx;
speech_rx = tanh(speech_rx);
speech_rx = 0.98 * speech_rx / (max(abs(speech_rx)) + 1e-12);

audiowrite(fullfile(outDir,'reconstructed_from_usrp.wav'),speech_rx,cfg.fs_speech);
save(fullfile(outDir,'final_results.mat'),'cfg','tx_bits','tx_coded','rx_bits','rx_coded','ber_usrp','ber_coded','diag_usrp','speech_rx','speech_8k');

make_wireless_plots_local(tx_packet,rx_buffer,rx_bits,tx_bits,diag_usrp,speech_8k,speech_rx,cfg,outDir);

fprintf('\nSaved key demo files in outputs/:\n');
fprintf('  modulation_demo.png\n');
fprintf('  bpsk_simulation_ber_constellation.png\n');
fprintf('  usrp_wireless_diagnostics.png\n');
fprintf('  usrp_rx_spectrum.png\n');
fprintf('  reconstructed_from_usrp.wav\n');
% Play the three signals back to back: corrupted -> cleaned -> wireless
fprintf('\nPlaying the three signals back to back:\n');
fprintf('  (1/3) Corrupted input speech (48 kHz)...\n');
soundsc(x_corrupt,cfg.fs_input);
pause(length(x_corrupt)/cfg.fs_input + 0.7);
fprintf('  (2/3) Prepared / cleaned speech (8 kHz)...\n');
soundsc(speech_8k,cfg.fs_speech);
pause(length(speech_8k)/cfg.fs_speech + 0.7);
fprintf('  (3/3) Wireless reconstructed speech (8 kHz)...\n');
soundsc(speech_rx,cfg.fs_speech);
pause(length(speech_rx)/cfg.fs_speech + 0.7);

%% ======================================================================
%                              FUNCTIONS
% ======================================================================

function y = signal_preparation_local(x,cfg)
    x = double(x(:));
    b = fir1(180,3400/(cfg.fs_input/2),'low',hamming(181));
    xf = filtfilt(b,1,x);
    y = resample(xf,cfg.fs_speech,cfg.fs_input);
    y = y(:)/(max(abs(y))+1e-12);
    fprintf('[signal_preparation] %d samples @ %d Hz -> %d samples @ %d Hz\n',length(x),cfg.fs_input,length(y),cfg.fs_speech);
end

function params = lpc_lsf_analysis_local(speech,cfg)
    speech = double(speech(:));
    speech_pe = filter([1 -cfg.pre_emph],1,speech);

    N = floor((length(speech_pe)-cfg.frame_length)/cfg.frame_shift)+1;
    p = cfg.lpc_order;
    win = hamming(cfg.frame_length,'periodic');
    default_lsf = linspace(pi/(p+1),pi*p/(p+1),p);

    params.lsf = repmat(default_lsf,N,1);
    params.gain = zeros(N,1);
    params.pitch = ones(N,1)*round((cfg.pitch_min+cfg.pitch_max)/2);
    params.vuv = zeros(N,1);
    params.num_frames = N;
    params.lpc_order = p;
    params.frame_length = cfg.frame_length;
    params.frame_shift = cfg.frame_shift;

    for i = 1:N
        idx = (i-1)*cfg.frame_shift + (1:cfg.frame_length);
        frame = speech_pe(idx);
        fw = frame .* win;

        % Frame gain
        params.gain(i) = sqrt(mean(frame.^2) + 1e-12);

        % LPC -> LSF
        try
            a = lpc(fw,p);
            lsf = poly2lsf(a);
            if numel(lsf)==p && all(lsf>0) && all(lsf<pi) && all(diff(lsf)>1e-6)
                params.lsf(i,:) = lsf(:).';
            end
        catch
            % Keep default LSF if LPC/LSF conversion fails
        end

        % Pitch estimation using normalized autocorrelation
        acf = xcorr(frame,'coeff');
        mid = cfg.frame_length;
        search_range = acf(mid+cfg.pitch_min:mid+cfg.pitch_max);
        [pk,ii] = max(search_range);
        params.pitch(i) = ii + cfg.pitch_min - 1;

        % Voiced/unvoiced decision using energy, autocorrelation peak, and ZCR
        frame_energy = sum(frame.^2);
        zcr = sum(abs(diff(sign(frame))))/(2*cfg.frame_length);

        if frame_energy > 1e-8 && pk > 0.30 && zcr < 0.42
            params.vuv(i) = 1;
        else
            params.vuv(i) = 0;
        end
    end

    % Smooth pitch contour after pitch and V/UV detection
    voiced_idx = params.vuv > 0.5;
    if sum(voiced_idx) >= 5
        params.pitch(voiced_idx) = round(smoothdata(params.pitch(voiced_idx), 'movmedian', 5));
    end

    % Remove sudden pitch jumps between adjacent voiced frames
    for ii = 2:params.num_frames
        if params.vuv(ii) > 0.5 && params.vuv(ii-1) > 0.5
            if abs(params.pitch(ii) - params.pitch(ii-1)) > 25
                params.pitch(ii) = params.pitch(ii-1);
            end
        end
    end

    fprintf('[lpc_lsf_analysis] %d frames, order %d\n',N,p);
end

function [bits,q] = quantize_parameters_local(params,cfg)
    N = params.num_frames;
    p = cfg.lpc_order;
    bits = zeros(N*cfg.bits_per_frame,1,'uint8');
    q = params;
    ptr = 1;

    % Log-domain gain quantization is more robust than linear gain quantization.
    gain_min_db = -50;
    gain_max_db = 0;

    for i = 1:N
        % V/UV flag
        bits(ptr) = uint8(params.vuv(i) > 0.5);
        ptr = ptr + 1;

        % Pitch period
        pitch_val = max(cfg.pitch_min, min(cfg.pitch_max, round(params.pitch(i))));
        pitch_idx = round((pitch_val - cfg.pitch_min) / (cfg.pitch_max - cfg.pitch_min) * (2^cfg.pitch_bits - 1));
        pitch_idx = max(0, min(2^cfg.pitch_bits - 1, pitch_idx));
        bits(ptr:ptr+cfg.pitch_bits-1) = int_to_bits_local(pitch_idx, cfg.pitch_bits);
        ptr = ptr + cfg.pitch_bits;

        % Gain in dB
        gain_db = 20*log10(params.gain(i) + 1e-12);
        gain_db = max(gain_min_db, min(gain_max_db, gain_db));
        gain_idx = round((gain_db - gain_min_db) / (gain_max_db - gain_min_db) * (2^cfg.gain_bits - 1));
        gain_idx = max(0, min(2^cfg.gain_bits - 1, gain_idx));
        bits(ptr:ptr+cfg.gain_bits-1) = int_to_bits_local(gain_idx, cfg.gain_bits);
        ptr = ptr + cfg.gain_bits;

        % LSF parameters
        for k = 1:p
            b = cfg.lsf_bit_allocation(k);
            lsf_val = max(0, min(pi, params.lsf(i,k)));
            lsf_idx = round(lsf_val/pi * (2^b - 1));
            lsf_idx = max(0, min(2^b - 1, lsf_idx));
            bits(ptr:ptr+b-1) = int_to_bits_local(lsf_idx, b);
            ptr = ptr + b;
        end
    end

    fprintf('[quantize_parameters] %d frames x %d bits/frame = %d bits\n',N,cfg.bits_per_frame,length(bits));
end

function params = dequantize_parameters_local(bits,N,cfg)
    bits=uint8(bits(:)); expected=N*cfg.bits_per_frame;
    if length(bits)<expected, bits=[bits;zeros(expected-length(bits),1,'uint8')]; end
    bits=bits(1:expected); p=cfg.lpc_order; ptr=1;
    params.lsf=zeros(N,p); params.gain=zeros(N,1); params.pitch=zeros(N,1); params.vuv=zeros(N,1);
    params.num_frames=N; params.lpc_order=p; params.frame_length=cfg.frame_length; params.frame_shift=cfg.frame_shift;
    for i=1:N
        params.vuv(i)=double(bits(ptr)); ptr=ptr+1;
        piq=bits_to_int_local(bits(ptr:ptr+cfg.pitch_bits-1)); ptr=ptr+cfg.pitch_bits;
        params.pitch(i)=round(piq*(cfg.pitch_max-cfg.pitch_min)/(2^cfg.pitch_bits-1)+cfg.pitch_min);
        giq=bits_to_int_local(bits(ptr:ptr+cfg.gain_bits-1)); ptr=ptr+cfg.gain_bits;
        gain_min_db = -50;
        gain_max_db = 0;
        gain_db = giq/(2^cfg.gain_bits-1)*(gain_max_db-gain_min_db) + gain_min_db;
        params.gain(i)=10^(gain_db/20);
        for k=1:p
            b=cfg.lsf_bit_allocation(k); liq=bits_to_int_local(bits(ptr:ptr+b-1)); ptr=ptr+b;
            params.lsf(i,k)=liq*pi/(2^b-1);
        end
    end
end

function y = speech_synthesis_local(params,cfg)
    rng(1);   % makes noise excitation repeatable

    N = params.num_frames;
    p = params.lpc_order;
    L = cfg.frame_length;
    H = cfg.frame_shift;

    y = zeros((N-1)*H + L,1);
    wsum = zeros(size(y));
    win = hann(L,'periodic');
    default_lsf = linspace(pi/(p+1),pi*p/(p+1),p);

    for i = 1:N
        % LSF -> stable LPC synthesis filter
        lsf = sort(params.lsf(i,:));
        lsf = max(1e-4,min(pi-1e-4,lsf));
        for k = 2:p
            if lsf(k) <= lsf(k-1)
                lsf(k) = min(pi-1e-4,lsf(k-1)+1e-4);
            end
        end
        if any(diff(lsf)<=0)
            lsf = default_lsf;
        end

        try
            a = lsf2poly(lsf);
            % Bandwidth expansion: improves filter stability and reduces harshness
            gamma = 0.985;
            for kk = 2:length(a)
                a(kk) = a(kk) * gamma^(kk-1);
            end
        catch
            a = [1 zeros(1,p)];
        end

        gain = max(params.gain(i),0);
        pitch = max(1,round(params.pitch(i)));

        if params.vuv(i) > 0.5
            % Voiced excitation: smoothed impulse train
            exc = zeros(L,1);
            pulse_idx = find(mod(0:L-1,pitch)==0);
            exc(pulse_idx) = 1.4 * gain * sqrt(pitch);

            glottal = hann(9);
            glottal = glottal / sum(glottal);
            exc = conv(exc,glottal,'same');

            % Very small breath noise to avoid overly buzzy synthesis
            exc = exc + 0.015 * gain * randn(L,1);
        else
            % Unvoiced excitation: lower-power filtered noise
            exc = 0.20 * gain * randn(L,1);
            exc = lowpass(exc,2800,cfg.fs_speech);
        end

        synth = filter(1,a,exc);
        pos = (i-1)*H + (1:L);
        y(pos) = y(pos) + synth .* win;
        wsum(pos) = wsum(pos) + win;
    end

    valid = wsum > 0.01;
    y(valid) = y(valid) ./ wsum(valid);

    % Inverse pre-emphasis and safe normalization
    y = filter(1,[1 -cfg.pre_emph],y);
    y = 0.95 * y / (max(abs(y)) + 1e-12);

    fprintf('[speech_synthesis] %d samples reconstructed\n',length(y));
end

function [tx,rrc,full_bits,symbols] = bpsk_modulate_local(data_bits,cfg)
    full_bits=[uint8(cfg.preamble_bits(:));uint8(data_bits(:))]; symbols=2*double(full_bits)-1;
    rrc=rcosdesign(cfg.rolloff,cfg.filter_span,cfg.samples_per_symbol,'sqrt').';
    tx=upfirdn(symbols,rrc,cfg.samples_per_symbol,1); tx=tx/(max(abs(tx))+1e-12); tx=complex(tx,zeros(size(tx)));
    tx=[tx;complex(zeros(2*cfg.filter_span*cfg.samples_per_symbol,1),0)];
    fprintf('[bpsk_modulate] %d data bits + %d preamble bits -> %d IQ samples\n',length(data_bits),length(cfg.preamble_bits),length(tx));
end

function [rx_bits,ber,d] = bpsk_demodulate_local(rx_signal,tx_bits,preamble_bits,sps,rrc)
    rx_signal=rx_signal(:); tx_bits=double(tx_bits(:)); preamble_bits=double(preamble_bits(:));
    pre_syms=2*preamble_bits-1; pre_syms=pre_syms(:); plen=length(pre_syms); data_len=length(tx_bits);
    rxf=conv(rx_signal,rrc(:),'same');
    best_score=-inf; best_bits=[]; best_data=[]; best_pre=[]; d=struct();
    for off=1:sps
        syms=rxf(off:sps:end); syms=syms(:);
        if length(syms)<plen+data_len, continue; end
        c=conv(syms,flipud(conj(pre_syms)),'valid'); m=abs(c); [~,peaks]=maxk(m,min(100,length(m)));
        for pp=1:length(peaks)
            si=peaks(pp); ds=si+plen; de=ds+data_len-1; if de>length(syms), continue; end
            rxpre=syms(si:si+plen-1); normcorr=abs(pre_syms'*rxpre)/(norm(pre_syms)*norm(rxpre)+1e-12);
            h=(pre_syms'*rxpre)/(pre_syms'*pre_syms+1e-12); if abs(h)<1e-12, continue; end
            eq=syms/h; preeq=eq(si:si+plen-1); preb=double(real(preeq)>0); preber=sum(preb~=preamble_bits)/plen;
            preb_inv=1-preb; preber_inv=sum(preb_inv~=preamble_bits)/plen; inverted=false;
            if preber_inv<preber, preber=preber_inv; inverted=true; end
            data_syms=eq(ds:de); bhat=double(real(data_syms)>0); if inverted, bhat=1-bhat; end
            epre=preeq-pre_syms; snrest=10*log10(mean(abs(pre_syms).^2)/(mean(abs(epre).^2)+1e-12));
            score=100*normcorr - 50*preber + snrest/20;
            if score>best_score
                best_score=score; best_bits=bhat; best_data=data_syms; best_pre=preeq;
                d.offset=off; d.sync=si; d.phase_deg=angle(h)*180/pi; d.norm_corr=normcorr; d.pre_ber=preber; d.inverted=inverted; d.snr_est_db=snrest;
            end
        end
    end
    if isempty(best_bits), error('BPSK demodulation failed: no valid packet found.'); end
    rx_bits=uint8(best_bits); ber=sum(double(rx_bits)~=tx_bits)/data_len; d.rx_data_syms=best_data; d.rx_pre_eq=best_pre;
    fprintf('[bpsk_demodulate] offset=%d | sync=%d | phase=%.1f deg | normCorr=%.3f | preBER=%.4f | SNRest=%.2f dB | inverted=%d\n',d.offset,d.sync,d.phase_deg,d.norm_corr,d.pre_ber,d.snr_est_db,d.inverted);
    fprintf('[bpsk_demodulate] Data BER = %.6g (%d/%d errors)\n',ber,sum(double(rx_bits)~=tx_bits),data_len);
end

function y = add_awgn_local(x,snr_db)
    p=mean(abs(x).^2); np=p/(10^(snr_db/10)); y=x+sqrt(np/2)*(randn(size(x))+1j*randn(size(x)));
end

function coded = ecc_encode_local(bits,cfg)
    % Forward error correction encoder. Protects the speech-parameter bit
    % stream before BPSK so that a few wireless bit flips can be corrected.
    bits = double(bits(:));
    switch lower(string(cfg.ecc_type))
        case "none"
            coded = uint8(bits);
        case "hamming"
            pad  = mod(-numel(bits),4);                 % pad to multiple of 4
            msg  = [bits; zeros(pad,1)];
            coded = encode(msg,7,4,'hamming/binary');
            coded = uint8(coded(:));
        case "conv"
            tail = round(log2(cfg.conv_trellis.numStates)); % memory length = K-1
            msg  = [bits; zeros(tail,1)];               % zero-tail termination
            coded = convenc(msg,cfg.conv_trellis);
            coded = uint8(coded(:));
        otherwise
            error('Unknown cfg.ecc_type "%s". Use "conv", "hamming", or "none".',cfg.ecc_type);
    end
end

function bits = ecc_decode_local(coded,msg_len,cfg)
    % Inverse of ecc_encode_local. Returns exactly msg_len message bits.
    coded = double(coded(:));
    switch lower(string(cfg.ecc_type))
        case "none"
            if numel(coded)<msg_len, coded=[coded;zeros(msg_len-numel(coded),1)]; end
            bits = coded(1:msg_len);
        case "hamming"
            nblk = 7*ceil(msg_len/4);
            if numel(coded)<nblk, coded=[coded;zeros(nblk-numel(coded),1)]; end
            coded = coded(1:nblk);
            dec  = decode(coded,7,4,'hamming/binary');
            bits = dec(1:msg_len);
        case "conv"
            tail   = round(log2(cfg.conv_trellis.numStates));
            ncoded = 2*(msg_len+tail);                  % rate 1/2 -> 2 coded bits per message bit
            if numel(coded)<ncoded, coded=[coded;zeros(ncoded-numel(coded),1)]; end
            coded = coded(1:ncoded);
            dec  = vitdec(coded,cfg.conv_trellis,cfg.conv_tblen,'term','hard');
            bits = dec(1:msg_len);
        otherwise
            error('Unknown cfg.ecc_type "%s". Use "conv", "hamming", or "none".',cfg.ecc_type);
    end
    bits = uint8(bits(:));
end
function bits=int_to_bits_local(v,n), bits=zeros(n,1,'uint8'); for k=1:n, p=2^(n-k); bits(k)=uint8(v>=p); if v>=p, v=v-p; end, end, end
function v=bits_to_int_local(bits), bits=double(bits(:).'); v=0; n=numel(bits); for k=1:n, v=v+bits(k)*2^(n-k); end, end

function make_pre_usrp_plots_local(x48,x8,sp_local,params,tx_bits,tx_packet,tx_symbols,snr_values,ber_values,ber_values_dec,diag_ex,cfg,outDir)
    % --- cosmetic style palette (visual only, does not affect any result) ---
    cBlue=[0.00 0.45 0.74]; cRed=[0.85 0.33 0.10]; cGreen=[0.20 0.66 0.33];
    cPurple=[0.49 0.18 0.56]; cGray=[0.40 0.40 0.40]; figBg=[0.96 0.97 1.00];

    figure('Name','DSP Summary','Color',figBg,'Position',[80 80 1200 800]);
    subplot(3,2,1); plot((0:length(x48)-1)/cfg.fs_input,x48,'Color',cRed,'LineWidth',1.0); grid on; title('Corrupted Input at 48 kHz'); xlabel('Time (s)'); ylabel('Amp');
    subplot(3,2,2); plot((0:length(x8)-1)/cfg.fs_speech,x8,'Color',cBlue,'LineWidth',1.0); grid on; title('Prepared Speech at 8 kHz'); xlabel('Time (s)'); ylabel('Amp');
    subplot(3,2,3); spectrogram(x8,hamming(128),64,256,cfg.fs_speech,'yaxis'); colormap(gca,turbo); title('Prepared Speech Spectrogram');
    ft=(0:params.num_frames-1)*cfg.frame_shift/cfg.fs_speech;
    subplot(3,2,4); plot(ft,params.pitch,'o-','Color',cPurple,'MarkerFaceColor',cPurple,'MarkerSize',4,'LineWidth',1.2); grid on; title('Pitch Period'); xlabel('Time (s)'); ylabel('Samples');
    subplot(3,2,5); stairs(ft,params.vuv,'Color',cGreen,'LineWidth',1.8); grid on; ylim([-0.2 1.2]); title('Voiced/Unvoiced'); xlabel('Time (s)'); ylabel('V/UV');
    subplot(3,2,6); bar([cfg.vuv_bits,cfg.pitch_bits,cfg.gain_bits,sum(cfg.lsf_bit_allocation)],'FaceColor',cBlue,'EdgeColor','none'); set(gca,'XTickLabel',{'V/UV','Pitch','Gain','LSF'}); grid on; ylabel('Bits/frame'); title(sprintf('%d bits/frame, %d data bits',cfg.bits_per_frame,length(tx_bits)));
    saveas(gcf,fullfile(outDir,'dsp_summary.png'));

    figure('Name','Modulation Demo','Color',figBg,'Position',[80 80 1200 800]);
    subplot(2,2,1); stem(double(tx_bits(1:min(100,end))),'filled','Color',cBlue,'MarkerFaceColor',cBlue); grid on; ylim([-0.2 1.2]); title('First DSP Data Bits'); xlabel('Bit index'); ylabel('Bit');
    n=min(3000,length(tx_packet)); subplot(2,2,2); plot((0:n-1)/cfg.usrp_sample_rate*1e3,real(tx_packet(1:n)),'Color',cRed,'LineWidth',1.0); grid on; title('Modulated BPSK Signal Just Before Transmission'); xlabel('Time (ms)'); ylabel('Amplitude');
    subplot(2,2,3); scatter(tx_symbols(1:min(2000,end)),zeros(min(2000,length(tx_symbols)),1),12,cPurple,'filled'); grid on; xlim([-1.5 1.5]); ylim([-0.5 0.5]); title('Ideal BPSK Constellation'); xlabel('I'); ylabel('Q');
    Nfft=4096; X=fftshift(abs(fft(tx_packet,Nfft)).^2); f=(-Nfft/2:Nfft/2-1)*(cfg.usrp_sample_rate/Nfft); subplot(2,2,4); plot(f/1e3,10*log10(X+1e-12),'Color',cGreen,'LineWidth',1.0); grid on; title('TX Signal Spectrum'); xlabel('Frequency (kHz)'); ylabel('Power (dB)');
    saveas(gcf,fullfile(outDir,'modulation_demo.png'));

    figure('Name','BPSK BER and Constellation','Color',figBg,'Position',[100 100 1200 500]);
    subplot(1,2,1); theory=0.5*erfc(sqrt(10.^(snr_values/10))); semilogy(snr_values,max(ber_values,1e-7),'o-','Color',cBlue,'MarkerFaceColor',cBlue,'LineWidth',2); hold on; semilogy(snr_values,max(ber_values_dec,1e-7),'s-','Color',cRed,'MarkerFaceColor',cRed,'LineWidth',2); semilogy(snr_values,max(theory,1e-7),'--','Color',cGray,'LineWidth',1.5); grid on; xlabel('SNR (dB)'); ylabel('BER'); legend('Uncoded (channel)','Coded (after ECC)','Theoretical BPSK','Location','southwest'); title('BER vs SNR with ECC');
    subplot(1,2,2); if ~isempty(diag_ex) && isfield(diag_ex,'rx_data_syms'), z=diag_ex.rx_data_syms(1:min(1500,end)); scatter(real(z),imag(z),10,cPurple,'filled'); grid on; title('Example Equalized Constellation'); xlabel('I'); ylabel('Q'); end
    saveas(gcf,fullfile(outDir,'bpsk_simulation_ber_constellation.png'));
end

function make_wireless_plots_local(tx_packet,rx_buffer,rx_bits,tx_bits,d,speech_8k,speech_rx,cfg,outDir)
    % --- cosmetic style palette (visual only, does not affect any result) ---
    cBlue=[0.00 0.45 0.74]; cRed=[0.85 0.33 0.10]; cGreen=[0.20 0.66 0.33];
    cPurple=[0.49 0.18 0.56]; figBg=[0.96 0.97 1.00];

    figure('Name','USRP Wireless Diagnostics','Color',figBg,'Position',[80 80 1200 900]);
    subplot(4,1,1); n=min(3000,length(tx_packet)); plot(real(tx_packet(1:n)),'Color',cRed,'LineWidth',1.0); grid on; title('Modulated Signal Just Before Transmission'); xlabel('Sample'); ylabel('Amp');
    subplot(4,1,2); n=min(20000,length(rx_buffer)); plot(real(rx_buffer(1:n)),'Color',cBlue,'LineWidth',0.8); grid on; title('Received Signal Before Demodulation - Real'); xlabel('Sample'); ylabel('Amp');
    subplot(4,1,3); plot(imag(rx_buffer(1:n)),'Color',cGreen,'LineWidth',0.8); grid on; title('Received Signal Before Demodulation - Imag'); xlabel('Sample'); ylabel('Amp');
    subplot(4,1,4); stem(double(rx_bits(1:min(100,end))),'filled','Color',cPurple,'MarkerFaceColor',cPurple); grid on; ylim([-0.2 1.2]); title(sprintf('First Recovered Bits | BER %.4g | SNR est %.2f dB',sum(double(rx_bits)~=double(tx_bits))/length(tx_bits),d.snr_est_db)); xlabel('Bit index'); ylabel('Bit');
    saveas(gcf,fullfile(outDir,'usrp_wireless_diagnostics.png'));

    figure('Name','Wireless Constellation and Spectrum','Color',figBg,'Position',[100 100 1200 500]);
    subplot(1,2,1); z=d.rx_data_syms(1:min(2000,end)); scatter(real(z),imag(z),10,cPurple,'filled'); grid on; title(sprintf('Wireless Equalized BPSK Constellation | normCorr %.3f',d.norm_corr)); xlabel('I'); ylabel('Q');
    subplot(1,2,2); Nfft=8192; r=rx_buffer(:); if length(r)>Nfft, r=r(1:Nfft); end; R=fftshift(abs(fft(r,Nfft)).^2); f=(-Nfft/2:Nfft/2-1)*(cfg.usrp_sample_rate/Nfft); plot(f/1e3,10*log10(R+1e-12),'Color',cBlue,'LineWidth',1.0); grid on; title('Received Wireless Spectrum Before Demodulation'); xlabel('Frequency (kHz)'); ylabel('Power (dB)');
    saveas(gcf,fullfile(outDir,'usrp_rx_spectrum.png'));

    figure('Name','Final Speech Reconstruction','Color',figBg,'Position',[100 100 1000 600]);
    n=min(length(speech_8k),length(speech_rx)); subplot(2,1,1); plot((0:n-1)/cfg.fs_speech,speech_8k(1:n),'Color',cBlue,'LineWidth',1.0); hold on; plot((0:n-1)/cfg.fs_speech,speech_rx(1:n),'--','Color',cRed,'LineWidth',1.0); grid on; legend('Prepared speech','USRP reconstructed'); title('Speech Comparison'); xlabel('Time (s)'); ylabel('Amplitude');
    subplot(2,1,2); spectrogram(speech_rx,hamming(128),64,256,cfg.fs_speech,'yaxis'); colormap(gca,turbo); title('Spectrogram of Reconstructed Speech');
    saveas(gcf,fullfile(outDir,'final_speech_reconstruction.png'));
end

function make_speech_quality_plot_local(speech_clean,speech_noecc,speech_ecc,snr_demo,cfg,outDir)
    % Side-by-side spectrograms: clean reference on top, then for each SNR the
    % reconstruction WITHOUT ECC (left) versus WITH ECC (right).
    figBg=[0.96 0.97 1.00];
    nrows = 1 + numel(snr_demo);
    figure('Name','Reconstructed Speech vs SNR (ECC off vs on)','Color',figBg,'Position',[60 40 1200 220*nrows]);
    subplot(nrows,2,[1 2]); spectrogram(speech_clean,hamming(128),64,256,cfg.fs_speech,'yaxis'); colormap(gca,turbo); title('Prepared speech (clean reference)');
    for k = 1:numel(snr_demo)
        subplot(nrows,2,2*k+1);
        if ~isempty(speech_noecc{k}), spectrogram(speech_noecc{k},hamming(128),64,256,cfg.fs_speech,'yaxis'); colormap(gca,turbo); else, axis off; end
        title(sprintf('SNR %+d dB  |  WITHOUT ECC',snr_demo(k)));
        subplot(nrows,2,2*k+2);
        if ~isempty(speech_ecc{k}), spectrogram(speech_ecc{k},hamming(128),64,256,cfg.fs_speech,'yaxis'); colormap(gca,turbo); else, axis off; end
        title(sprintf('SNR %+d dB  |  WITH ECC',snr_demo(k)));
    end
    saveas(gcf,fullfile(outDir,'speech_quality_vs_snr.png'));
end


function serialNum = auto_detect_usrp_serial_local(platform)
    serialNum = "";

    % Try findsdru first, because it is the common MATLAB USRP discovery command.
    radios = [];
    try
        radios = findsdru;
    catch
        % If findsdru is unavailable, try findsdr.
        try
            radios = findsdr;
        catch ME
            error('Could not run findsdru or findsdr. Check the USRP support package. Original error: %s', ME.message);
        end
    end

    % Parse structure-array output.
    if isstruct(radios)
        for ii = 1:numel(radios)
            candidateSerial = "";
            candidatePlatform = "";
            candidateStatus = "";

            if isfield(radios, 'SerialNum')
                candidateSerial = string(radios(ii).SerialNum);
            end
            if isfield(radios, 'Platform')
                candidatePlatform = string(radios(ii).Platform);
            end
            if isfield(radios, 'Status')
                candidateStatus = string(radios(ii).Status);
            end

            serialOK = strlength(candidateSerial) > 0;
            platformOK = strlength(candidatePlatform) == 0 || contains(upper(candidatePlatform), upper(string(platform))) || contains(upper(string(platform)), upper(candidatePlatform));
            statusOK = strlength(candidateStatus) == 0 || contains(lower(candidateStatus), "success") || contains(lower(candidateStatus), "available");

            if serialOK && platformOK && statusOK
                serialNum = candidateSerial;
                return;
            end
        end
    end

    % Parse table output, used by some MATLAB installs.
    if istable(radios)
        varNames = string(radios.Properties.VariableNames);
        serialCol = find(strcmpi(varNames, "SerialNum") | strcmpi(varNames, "SerialNumber") | strcmpi(varNames, "Serial"), 1);
        platformCol = find(strcmpi(varNames, "Platform"), 1);
        statusCol = find(strcmpi(varNames, "Status"), 1);

        for ii = 1:height(radios)
            candidateSerial = "";
            candidatePlatform = "";
            candidateStatus = "";

            if ~isempty(serialCol)
                candidateSerial = string(radios{ii, serialCol});
            end
            if ~isempty(platformCol)
                candidatePlatform = string(radios{ii, platformCol});
            end
            if ~isempty(statusCol)
                candidateStatus = string(radios{ii, statusCol});
            end

            serialOK = strlength(candidateSerial) > 0;
            platformOK = strlength(candidatePlatform) == 0 || contains(upper(candidatePlatform), upper(string(platform))) || contains(upper(string(platform)), upper(candidatePlatform));
            statusOK = strlength(candidateStatus) == 0 || contains(lower(candidateStatus), "success") || contains(lower(candidateStatus), "available");

            if serialOK && platformOK && statusOK
                serialNum = candidateSerial;
                return;
            end
        end
    end

    % If discovery returned a single object/string-like result, try converting it.
    try
        radiosText = string(radios);
        token = regexp(join(radiosText, " "), '[A-Za-z0-9]{6,}', 'match', 'once');
        if ~isempty(token)
            serialNum = string(token);
            return;
        end
    catch
    end

    error(['No usable USRP serial number was detected. Run "findsdru" in the MATLAB command window, ', ...
           'copy the serial number it prints, and set cfg.usrp_serial to that value near the top of this file.']);
end
