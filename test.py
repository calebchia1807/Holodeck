import torch
import open_clip

# Load model and preprocessing function
model, _, preprocess = open_clip.create_model_and_transforms("ViT-B-32", pretrained="laion2b_s34b_b79k")
tokenizer = open_clip.get_tokenizer("ViT-B-32")  # ✅ Corrected model name

with torch.no_grad():
    queries = "chair"
    print(queries)
    query_feature_clip = model.encode_text(
        tokenizer(queries)
    )
    print("CLIP text encoding done")
