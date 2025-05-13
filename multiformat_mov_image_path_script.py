import os
import re
import shutil

# Paths
posts_dir = "/Users/matthewpye/Documents/blog/mjpye.github.io/content/posts/"
attachments_dir = "/Users/matthewpye/Documents/Obsidian_Vault/Create 3 Robot/attachments/"
static_images_dir = "/Users/matthewpye/Documents/blog/mjpye.github.io/static/images/"

# Supported media extensions
image_extensions = ('.png', '.jpg', '.jpeg', '.gif', '.svg')
video_extensions = ('.webm', '.mov')

# Step 1: Process each markdown file in the posts directory
for filename in os.listdir(posts_dir):
    if filename.endswith(".md"):
        filepath = os.path.join(posts_dir, filename)

        with open(filepath, "r") as file:
            content = file.read()

        # Step 2: Find all media links in the format ![[file.ext]] or [[file.ext]]
        media_files = re.findall(r'!?\\?\\?\[\[([^\]]+\.(?:png|jpg|jpeg|gif|svg|webm|mov))\]\]', content, re.IGNORECASE)

        # Step 3: Replace media links and ensure URLs are correctly formatted
        for media in media_files:
            ext = os.path.splitext(media)[1].lower()
            media_path = f"/images/{media.replace(' ', '%20')}"
            full_source_path = os.path.join(attachments_dir, media)

            if ext in video_extensions:
                # Markdown HTML fallback for video (autoplay, muted, loop)
                replacement = (
                    f'<video src="{media_path}" autoplay muted loop playsinline style="max-width:100%; height:auto;"></video>'
                )
            else:
                # Standard image markdown
                replacement = f"![Image Description]({media_path})"

            # Replace both [[file.ext]] and ![[file.ext]] safely
            content = re.sub(rf'!?\[\[{re.escape(media)}\]\]', replacement, content)

            # Step 4: Copy the media file to the Hugo static/images directory if it exists
            if os.path.exists(full_source_path):
                shutil.copy(full_source_path, static_images_dir)

        # Step 5: Write the updated content back to the markdown file
        with open(filepath, "w") as file:
            file.write(content)

print("Markdown files processed and media copied successfully.")

