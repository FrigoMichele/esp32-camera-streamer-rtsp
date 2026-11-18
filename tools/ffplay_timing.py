#!/usr/bin/env python3
"""Summarize a single FFplay -loglevel datetime+debug -fdebug ts session.

For this video-only RTSP stream, with showinfo BEFORE any timestamp filters.
Input is a text log, not video. All intervals use the receiver's clock.
"""

import argparse
from datetime import datetime
from math import ceil
import re
from statistics import median


PACKET = re.compile(r"\b(ff_read_packet|read_frame_internal) stream=0, pts=(-?\d+),")
PICTURE = re.compile(r"\[Parsed_showinfo_\d+ @ [^]]+\]\s+n:\s*\d+\s+pts:\s*(-?\d+)\b")
TIME_BASE = re.compile(r"\[Parsed_showinfo_\d+ @ [^]]+\] config in time_base: ([^,]+)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=argparse.FileType("r", encoding="utf-8-sig", errors="replace"))
    parser.add_argument("--skip", type=int, default=0, help="exclude first N matched pictures (warmup)")
    args = parser.parse_args()
    if args.skip < 0:
        parser.error("--skip must be nonnegative")

    last_read, parsed, decoded = {}, {}, {}
    time_bases = set()
    with args.log as lines:
        for line in lines:
            packet = PACKET.search(line)
            if packet:
                dest = last_read if packet[1] == "ff_read_packet" else parsed
                pts = int(packet[2])
            elif picture := PICTURE.search(line):
                dest, pts = decoded, int(picture[1])
            else:
                if base := TIME_BASE.search(line):
                    time_bases.add(base[1])
                continue
            try:
                dest[pts] = datetime.fromisoformat(line[:23])
            except ValueError:
                parser.error("missing PC timestamp: use -loglevel datetime+debug and redirect raw stderr")

    if time_bases != {"1/90000"}:
        parser.error("expected showinfo on the original H264/90000 time base; see docs/latency-analysis.md")
    # Decode order, not dictionary/set intersection order. Ignore NOPTS and
    # pictures without all three events (e.g. probing or stopping the player).
    matched = [p for p in decoded if p in last_read and p in parsed]
    frames = matched[args.skip:]
    if not frames:
        parser.error("no complete same-PTS samples after warmup; use one session and showinfo before setpts")
    print(f"Matched {len(matched)}/{len(decoded)} decoded pictures; reporting {len(frames)}")
    for label, start, end in (("last fragment read -> parsed AU", last_read, parsed),
                              ("parsed AU -> decoded filter", parsed, decoded),
                              ("last fragment read -> decoded filter", last_read, decoded)):
        pairs = sorted((round((end[p] - start[p]).total_seconds() * 1000), p) for p in frames)
        if pairs[0][0] < 0:
            parser.error("negative interval: check clock steps, PTS and session boundaries")
        ms = [v for v, _ in pairs]
        print(f"{label}: median={median(ms):g} p95={ms[ceil(.95 * len(ms)) - 1]} "
              f"max={ms[-1]} ms (max at pts={pairs[-1][1]})")


if __name__ == "__main__":
    main()
