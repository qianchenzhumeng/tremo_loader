fn main() -> shadow_rs::SdResult<()> {
    println!("cargo:rerun-if-changed=.");
    shadow_rs::new()
}
