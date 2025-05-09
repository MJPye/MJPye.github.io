import os
import re
import shutil

# Paths
posts_dir = "/Users/matthewpye/Documents/blog/mjpye.github.io/content/posts/"
attachments_dir = "/Users/matthewpye/Documents/Obsidian_Vault/Create 3 Robot/attachments/"
static_images_dir = "/Users/matthewpye/Documents/blog/mjpye.github.io/static/images/"

# Step 1: Process each markdown file in the posts directory
for filename in os.listdir(posts_dir):
    if filename.endswith(".md"):
        filepath = os.path.join(posts_dir, filename)
        
        with open(filepath, "r") as file:
            content = file.read()
        
        # Step 2: Find all image links in the format ![[image.png]] or [[image.png]]
        images = re.findall(r'!?\\?\\?\[\[([^\]]*\.png)\]\]', content)

        # Alternate more readable version:
        # images = re.findall(r'!?\[\[([^\]]+\.png)\]\]', content)

        # Step 3: Replace image links and ensure URLs are correctly formatted
        for image in images:
            # Prepare the Markdown-compatible link
            markdown_image = f"![Image Description](/images/{image.replace(' ', '%20')})"

            # Replace both [[image.png]] and ![[image.png]] safely
            content = re.sub(rf'!?\[\[{re.escape(image)}\]\]', markdown_image, content)
            
            # Step 4: Copy the image to the Hugo static/images directory if it exists
            image_source = os.path.join(attachments_dir, image)
            if os.path.exists(image_source):
                shutil.copy(image_source, static_images_dir)

        # Step 5: Write the updated content back to the markdown file
        with open(filepath, "w") as file:
            file.write(content)

print("Markdown files processed and images copied successfully.")

