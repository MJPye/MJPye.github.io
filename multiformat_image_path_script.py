import os
import re
import shutil

# Paths
posts_dir = "/Users/matthewpye/Documents/blog/mjpye.github.io/content/posts/"
attachments_dir = "/Users/matthewpye/Documents/Obsidian_Vault/Create 3 Robot/attachments/"
static_images_dir = "/Users/matthewpye/Documents/blog/mjpye.github.io/static/images/"

# Supported file extensions
extensions = ["png", "jpg", "jpeg", "svg", "gif", "webm"]

# Step 1: Process each markdown file in the posts directory
for filename in os.listdir(posts_dir):
    if filename.endswith(".md"):
        filepath = os.path.join(posts_dir, filename)

        with open(filepath, "r") as file:
            content = file.read()

        # Step 2: Find all matching media references like ![[image.png]] or [[image.png]]
        pattern = r'!?\[\[([^\]]+\.(?:' + '|'.join(extensions) + r'))\]\]'
        matches = re.findall(pattern, content, re.IGNORECASE)

        # Step 3: Replace each media reference with proper markdown/html
        for media in matches:
            ext = media.split('.')[-1].lower()
            encoded_media = media.replace(' ', '%20')
            media_path = f"/images/{encoded_media}"

            if ext == 'webm':
                replacement = f'<video src="{media_path}" autoplay loop muted playsinline style="max-width: 100%;" controls></video>'
            elif ext == 'gif':
                # <img> automatically loops GIFs; we use attributes for consistency
                replacement = f'<img src="{media_path}" alt="GIF" loading="lazy" style="max-width: 100%;">'
            else:
                replacement = f'![Image Description]({media_path})'

            # Replace any form: [[...]] or ![[...]]
            content = re.sub(rf'!?\[\[{re.escape(media)}\]\]', replacement, content)

            # Step 4: Copy media file if it exists
            media_source = os.path.join(attachments_dir, media)
            if os.path.exists(media_source):
                shutil.copy(media_source, static_images_dir)

        # Step 5: Save updated markdown file
        with open(filepath, "w") as file:
            file.write(content)

print("Markdown files processed and media copied successfully.")

