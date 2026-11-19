# プロトコルメモ

- Wi-Fi AP: `192.168.4.1/24`; client: static `192.168.4.2/24`
- UDP voice: port `4210`; 4-byte little-endian CoreS3-to-CoreS3 header (`magic`, 16-bit sequence), then 320 signed 16-bit mono samples at 16 kHz (20 ms). Two capture buffers cycle through FREE → QUEUED → READY → SENDING; Mic's release callback alone promotes a block to READY.
- TCP still image: port `4211`; 4-byte big-endian JPEG length, then JPEG bytes (max 512 KiB)
- PTT release calls `M5.Mic.end()` to stop/join capture, discard unfinished requests, and then clear any completed-but-unsent tail. Audio and image use separate sockets. PTT is a touch-and-hold zone on each display.
- This is an experimental starter protocol, not a hardened or interoperable network protocol.
