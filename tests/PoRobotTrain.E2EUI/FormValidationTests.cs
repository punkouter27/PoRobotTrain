using Microsoft.Playwright;

namespace PoRobotTrain.E2EUI;

/// <summary>FluentValidation in the trimmed Release build: errors show inline under the field, never as a dialog.</summary>
[Collection("browser")]
public sealed class FormValidationTests(BrowserFixture fx)
{
    [Fact]
    public async Task BadConnectionProfileShowsInlineErrorsAndIsNotSaved()
    {
        var (ctx, page, errors) = await fx.OpenAsync(1366, 900, advanced: true);
        try
        {
            await page.GotoAsync(BrowserFixture.BaseUrl + "/setup", new() { WaitUntil = WaitUntilState.NetworkIdle });
            var host = page.GetByRole(AriaRole.Textbox, new() { Name = "Host", Exact = true });
            await host.WaitForAsync(new() { Timeout = 20000 });
            await host.FillAsync("bad host!");
            await host.PressAsync("Tab");
            var port = page.Locator("label:has-text('Port') .rz-numeric input");
            await port.FillAsync("70000");
            await port.PressAsync("Tab");
            await page.GetByRole(AriaRole.Button, new() { Name = "Save profile" }).ClickAsync();

            await Assertions.Expect(page.Locator("label:has-text('Host') .field-error")).ToHaveTextAsync("Host: a hostname or IP address.");
            await Assertions.Expect(page.Locator("label:has-text('Port') .field-error")).ToHaveTextAsync("Port must be 1-65535.");
            await Assertions.Expect(page.GetByText("Saved profile")).ToHaveCountAsync(0);
            Assert.Empty(errors);
        }
        finally
        {
            await ctx.CloseAsync();
        }
    }

    [Fact]
    public async Task MotionWithBadNameShowsInlineError()
    {
        var (ctx, page, errors) = await fx.OpenAsync(1366, 900);
        try
        {
            await page.GotoAsync(BrowserFixture.BaseUrl + "/motions", new() { WaitUntil = WaitUntilState.NetworkIdle });
            await page.GetByRole(AriaRole.Button, new() { Name = "+ New motion" }).ClickAsync();
            var name = page.GetByLabel("Motion name (a-z, 0-9, _)");
            await name.FillAsync("Bad Name");
            await name.PressAsync("Tab");
            await page.GetByRole(AriaRole.Button, new() { Name = "Save", Exact = true }).ClickAsync();

            await Assertions.Expect(page.Locator(".field-error").First)
                .ToHaveTextAsync("Name: 1-40 lower-case letters, digits or _, starting with a letter or digit.");
            Assert.Empty(errors);
        }
        finally
        {
            await ctx.CloseAsync();
        }
    }
}
