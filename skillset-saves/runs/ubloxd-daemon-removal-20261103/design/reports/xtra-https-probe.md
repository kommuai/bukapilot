# Supplemental transport evidence: qcomgpsd XTRA assistance

## Status

This is a factual follow-up to `security-seed.json`. It records source behavior and a one-time host-side HEAD probe; it does not rate the current code as vulnerable or decide whether to change, authenticate, or disable XTRA assistance. No product code was changed, no tests were run, and no XTRA file was downloaded.

## Source behavior

Primary code evidence is `system/qcomgpsd/qcomgpsd.py` at SHA-256 `18a24b715e898642b89d20ed35550513fa54e292038a9b4a1b6e471d9e8b30a6`:

- `ASSISTANCE_URL` is `http://xtrapath3.izatcloud.net/xtra3grc.bin` (line 35).
- After `wait_for_modem()`, `main()` starts a separate `downloader_loop` process before constructing `ModemDiag` and calling `setup_quectel()` (lines 234–259).
- The downloader removes a pre-existing `/tmp/xtra3grc.bin`. If `QCOM_ALT_ASSISTANCE_PATH` names an existing file, it copies that file; otherwise it retries the HTTP request with a five-second timeout, waiting ten seconds between attempts (lines 100–128).
- The response body is streamed in 8,192-byte chunks. It aborts after the temporary file exceeds 100,000 bytes and renames the file to `/tmp/xtra3grc.bin` after the stream completes (lines 100–111). The code does not call `raise_for_status()`, check the response content type, or verify a digest or signature.
- `setup_quectel()` enables modem XTRA and, when the file exists, injects it through `mmcli --location-inject-assistance-data`, then removes the file (lines 132–136, 167–174).
- In `main()`, the presence of a downloaded file triggers setup/injection while assistance is wanted; publishing a GNSS fix sets the stop event for the downloader (lines 267–270, 361–364).

`system/manager/process_config.py:50-51,110` enables `qcomgpsd` on TICI or KA2 and currently selects it onroad when UBLOX hardware is not detected. That source configuration is already used for KA2; it does not establish approval or runtime equivalence for promoting qcomgpsd on TICI.

## One-time endpoint probe

Admiral recorded these HEAD-only host probes on 2026-11-03. The HTTPS probe used curl's normal certificate verification; no verification bypass was supplied. No response body was downloaded.

```text
curl -I -L --max-time 10 https://xtrapath3.izatcloud.net/xtra3grc.bin
HTTP 200 OK
Content-Type: application/octet-stream
Content-Length: 34335
Server: AmazonS3
Last-Modified: Tue, 03 Nov 2026 17:04:11 GMT
Date: Tue, 03 Nov 2026 17:07:54 GMT
CloudFront response headers were also present.
```

The same HEAD command against the current HTTP URL returned HTTP 200 with the same content type and content length. It did not redirect to HTTPS.

## Independent configuration corroboration

The pinned AOSP `gps.conf` for Huawei Angler lists `XTRA_SERVER_3=https://xtrapath3.izatcloud.net/xtra3grc.bin` ([source file at commit `7472411642e541f54e7314094a64ccc1fbccd984`](https://android.googlesource.com/device/huawei/angler/+/7472411642e541f54e7314094a64ccc1fbccd984/gps.conf), line 8). This shows that one Android client configuration names the HTTPS endpoint. It does not establish support or trust configuration in this vehicle's client.

## Limits of this evidence

The HEAD responses are a one-time observation from the host on the date above. They do not establish the vehicle's CA trust, future endpoint behavior, signed-file semantics, XTRA payload safety, or the endpoint's behavior for GET requests. The AOSP configuration is corroboration for an HTTPS URL only; it is not evidence that this client authenticates the response. The pending XTRA behavior choice remains with the design owner; this report makes no implementation choice.
