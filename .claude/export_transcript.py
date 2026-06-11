#!/usr/bin/env python
"""
Convert Claude Code JSONL transcript to readable Markdown.
Triggered by Stop hook — reads session_id from stdin, writes .md to transcripts/.
"""
import json
import os
import sys
from datetime import datetime


def extract_text_from_content(content):
    """Extract human-readable text from message content (list of blocks)."""
    if isinstance(content, str):
        return content

    parts = []
    if not isinstance(content, list):
        return str(content)

    for block in content:
        if not isinstance(block, dict):
            parts.append(str(block))
            continue

        block_type = block.get("type", "")

        if block_type == "text":
            parts.append(block.get("text", ""))

        elif block_type == "tool_use":
            name = block.get("name", "?")
            inp = block.get("input", {})
            # Summarize tool call
            if name == "Write":
                parts.append(f"\n> 📝 **Write** `{inp.get('file_path', '?')}`\n")
            elif name == "Edit":
                parts.append(f"\n> ✏️ **Edit** `{inp.get('file_path', '?')}`\n")
            elif name == "Bash":
                cmd = inp.get("command", "?")
                parts.append(f"\n> 💻 **Run** `{cmd[:120]}`\n")
            elif name == "Read":
                parts.append(f"\n> 📖 **Read** `{inp.get('file_path', '?')}`\n")
            elif name == "Grep":
                parts.append(f"\n> 🔍 **Grep** `{inp.get('pattern', '?')}`\n")
            elif name == "Glob":
                parts.append(f"\n> 🌐 **Glob** `{inp.get('pattern', '?')}`\n")
            else:
                parts.append(f"\n> 🔧 **{name}**\n")

        elif block_type == "tool_result":
            # Skip tool results to keep output clean
            is_error = block.get("is_error", False)
            if is_error:
                parts.append("\n> ⚠️ *Tool error*\n")

    return "\n".join(parts)


def convert_jsonl_to_md(jsonl_path, md_path):
    """Convert a JSONL transcript file to Markdown."""
    messages = []
    session_id = "unknown"
    started_at = None

    with open(jsonl_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue

            # Track session info
            if obj.get("sessionId"):
                session_id = obj["sessionId"]
            if obj.get("timestamp") and not started_at:
                started_at = obj["timestamp"]

            msg_type = obj.get("type", "")

            if msg_type == "user":
                content = obj.get("message", {}).get("content", "")
                text = extract_text_from_content(content)
                if text.strip():
                    messages.append(("user", text))

            elif msg_type == "assistant":
                content = obj.get("message", {}).get("content", "")
                text = extract_text_from_content(content)
                if text.strip():
                    messages.append(("assistant", text))

    if not messages:
        print("No messages found in transcript.")
        return

    # Build markdown
    lines = []
    lines.append(f"# Claude Code Session Transcript")
    lines.append(f"")
    lines.append(f"- **Session**: `{session_id}`")
    if started_at:
        try:
            dt = datetime.fromisoformat(started_at.replace("Z", "+00:00"))
            lines.append(f"- **Started**: {dt.strftime('%Y-%m-%d %H:%M:%S UTC')}")
        except ValueError:
            lines.append(f"- **Started**: {started_at}")
    lines.append(f"- **Messages**: {len(messages)}")
    lines.append(f"")
    lines.append("---")
    lines.append("")

    for role, text in messages:
        if role == "user":
            lines.append("## 🙋 You")
        else:
            lines.append("## 🤖 Claude")
        lines.append("")
        lines.append(text)
        lines.append("")
        lines.append("---")
        lines.append("")

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"Transcript saved to: {md_path}")
    print(f"  Messages: {len(messages)}")


def main():
    # Read session_id from stdin (provided by Stop hook)
    try:
        hook_input = json.load(sys.stdin)
    except (json.JSONDecodeError, Exception):
        hook_input = {}

    session_id = hook_input.get("session_id", "")
    if not session_id:
        # Fallback: find latest JSONL in project transcript dir
        print("No session_id in hook input, finding latest transcript...")

    home = os.path.expanduser("~")
    proj_slug = "D--STM32Project-H743-MicroROS-Test"
    transcript_dir = os.path.join(home, ".claude", "projects", proj_slug)

    # Find the transcript file
    jsonl_path = None
    if session_id:
        candidate = os.path.join(transcript_dir, f"{session_id}.jsonl")
        if os.path.exists(candidate):
            jsonl_path = candidate

    if not jsonl_path:
        # Find latest by modification time
        import glob
        candidates = glob.glob(os.path.join(transcript_dir, "*.jsonl"))
        if candidates:
            jsonl_path = max(candidates, key=os.path.getmtime)
            print(f"Using latest: {jsonl_path}")
        else:
            print("No transcript files found.", file=sys.stderr)
            sys.exit(1)

    # Output path: project/transcripts/ with timestamp
    project_dir = os.path.dirname(os.path.dirname(transcript_dir))  # not great, use env or hardcode
    # Better: use the script's location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)  # .claude/.. → project root
    out_dir = os.path.join(project_root, "transcripts")
    os.makedirs(out_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    md_filename = f"session_{timestamp}.md"
    md_path = os.path.join(out_dir, md_filename)

    convert_jsonl_to_md(jsonl_path, md_path)


if __name__ == "__main__":
    main()
