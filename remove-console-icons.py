#!/usr/bin/env python3
"""
Remove Icons/Emoji from Console.log in Frontend
Removes all emoji and icons from console.log statements in frontend folder
"""

import os
import re
from pathlib import Path

def remove_icons_from_console_logs(file_path):
    """Remove icons/emoji from console.log statements in a file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        original_content = content
        changes_made = 0
        
        # Pattern to match console.log with emoji/icons
        # This will match console.log('🟢 text') and replace with console.log('text')
        patterns = [
            # Match console.log with emoji at start of string
            (r"console\.log\('([🟢🔴🟡⚪🟠🔵🟣⚫⚪🔘🔲🔳⭕❌✅❓❗💡🚀🎯📊📈📉📋📌📍📎📏📐📑📒📓📔📕📖📗📘📙📚📛📜📝📞📟📠📡📢📣📤📥📦📧📨📩📪📫📬📭📮📯📰📱📲📳📴📵📶📷📸📹📺📻📼📽️📿🔀🔁🔂🔃🔄🔅🔆🔇🔈🔉🔊🔋🔌🔍🔎🔏🔐🔑🔒🔓🔔🔕🔖🔗🔘🔙🔚🔛🔜🔝🔞🔟🔠🔡🔢🔣🔤🔥🔦🔧🔨🔩🔪🔫🔬🔭🔮🔯🔰🔱🔲🔳🔴🔵🔶🔷🔸🔹🔺🔻🔼🔽🕐🕑🕒🕓🕔🕕🕖🕗🕘🕙🕚🕛🕜🕝🕞🕟🕠🕡🕢🕣🕤🕥🕦🕧🚫⚠️💓👁️🌐🧹➕➖🔊📢📭🔄]+)\\s*([^']*)'", r"console.log('\2'"),
            
            # Match console.log with emoji in template literals
            (r"console\.log\(`([🟢🔴🟡⚪🟠🔵🟣⚫⚪🔘🔲🔳⭕❌✅❓❗💡🚀🎯📊📈📉📋📌📍📎📏📐📑📒📓📔📕📖📗📘📙📚📛📜📝📞📟📠📡📢📣📤📥📦📧📨📩📪📫📬📭📮📯📰📱📲📳📴📵📶📷📸📹📺📻📼📽️📿🔀🔁🔂🔃🔄🔅🔆🔇🔈🔉🔊🔋🔌🔍🔎🔏🔐🔑🔒🔓🔔🔕🔖🔗🔘🔙🔚🔛🔜🔝🔞🔟🔠🔡🔢🔣🔤🔥🔦🔧🔨🔩🔪🔫🔬🔭🔮🔯🔰🔱🔲🔳🔴🔵🔶🔷🔸🔹🔺🔻🔼🔽🕐🕑🕒🕓🕔🕕🕖🕗🕘🕙🕚🕛🕜🕝🕞🕟🕠🕡🕢🕣🕤🕥🕦🕧🚫⚠️💓👁️🌐🧹➕➖🔊📢📭🔄]+)\\s*([^`]*)`", r"console.log(`\2`"),
        ]
        
        for pattern, replacement in patterns:
            new_content = re.sub(pattern, replacement, content, flags=re.MULTILINE)
            if new_content != content:
                changes_made += content.count(pattern.split('(')[1].split(')')[0]) - new_content.count(pattern.split('(')[1].split(')')[0])
                content = new_content
        
        # More specific patterns for common emoji
        emoji_patterns = [
            (r"console\.log\('🟢\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('🔴\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('📨\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('💓\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('📤\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('👁️\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('🌐\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('📊\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('🚫\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('📡\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('🧹\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('➕\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('➖\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('🔊\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('📢\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('📭\s*([^']*)'", r"console.log('\1'"),
            (r"console\.log\('🔄\s*([^']*)'", r"console.log('\1'"),
            
            # Template literals
            (r"console\.log\(`🟢\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`🔴\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`📨\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`💓\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`📤\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`👁️\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`🌐\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`📊\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`🚫\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`📡\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`🧹\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`➕\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`➖\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`🔊\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`📢\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`📭\s*([^`]*)`", r"console.log(`\1`"),
            (r"console\.log\(`🔄\s*([^`]*)`", r"console.log(`\1`"),
        ]
        
        for pattern, replacement in emoji_patterns:
            new_content = re.sub(pattern, replacement, content, flags=re.MULTILINE)
            if new_content != content:
                changes_made += 1
                content = new_content
        
        # Write back if changes were made
        if content != original_content:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
            return changes_made
        
        return 0
        
    except Exception as e:
        print(f"❌ Error processing {file_path}: {e}")
        return 0

def process_frontend_folder():
    """Process all TypeScript/JavaScript files in frontend folder"""
    print("🧹 REMOVING ICONS FROM CONSOLE.LOG STATEMENTS")
    print("=" * 50)
    
    frontend_path = Path('frontend')
    if not frontend_path.exists():
        print("❌ Frontend folder not found!")
        return
    
    # File extensions to process
    extensions = ['.ts', '.tsx', '.js', '.jsx']
    
    # Folders to skip
    skip_folders = {'.next', 'node_modules', 'dist', 'build'}
    
    total_files = 0
    total_changes = 0
    processed_files = []
    
    for file_path in frontend_path.rglob('*'):
        # Skip directories and non-target files
        if file_path.is_dir():
            continue
            
        # Skip files in excluded folders
        if any(skip_folder in file_path.parts for skip_folder in skip_folders):
            continue
            
        # Only process target file extensions
        if file_path.suffix not in extensions:
            continue
        
        print(f"🔍 Processing: {file_path}")
        changes = remove_icons_from_console_logs(file_path)
        
        if changes > 0:
            print(f"  ✅ Removed {changes} icons")
            processed_files.append((str(file_path), changes))
            total_changes += changes
        else:
            print(f"  ⚪ No icons found")
        
        total_files += 1
    
    # Summary
    print(f"\n📊 SUMMARY:")
    print(f"Files processed: {total_files}")
    print(f"Files with changes: {len(processed_files)}")
    print(f"Total icons removed: {total_changes}")
    
    if processed_files:
        print(f"\n📝 FILES MODIFIED:")
        for file_path, changes in processed_files:
            print(f"  - {file_path}: {changes} icons removed")
    
    if total_changes > 0:
        print(f"\n✅ SUCCESS: Removed {total_changes} icons from console.log statements!")
        print(f"💡 Your console output will now be cleaner")
    else:
        print(f"\n✅ No icons found in console.log statements")

if __name__ == "__main__":
    process_frontend_folder()